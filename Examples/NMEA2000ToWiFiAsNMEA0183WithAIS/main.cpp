/*
Demo: NMEA2000 library. Sends NMEA2000 to WiFi in NMEA0183 format, including AIS Sentences 1, 5, 18, 24.
Example from Timo Lappalainen, extended by Ronnie Zeiller

The code has been tested with ESP32.

  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define CONFIG_ESP32_ENABLE_COREDUMP_TO_UART
#define CONFIG_ESP32_PANIC_GDBSTUB

#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#define GPIO_CAN_DISABLE GPIO_NUM_16  // -> CAN Transceiver MCP 2562 Pin 8 STBY
#define LED_BUILTIN 13

// CONFIG
#define ENABLE_N2K_ON_USB 0       // Writes N2k PGNs to Serial (USB)
#define N2K_TEXTMODE      1       // If 1 -> Text-Modus, 0 -> Actisense Format
#define ENABLE_NMEA0183_ON_USB 0  // Writes NMEA0183 + AIS to Serial (USB)

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <memory>
#include "N2kDataToNMEA0183.h"
#include "List.h"
#include "BoardSerialNumber.h"

// Wifi AP
const char *ssid = "SSID";
const char *password = "Passphrase";
const uint16_t ServerPort=2222; // Define the port, where served sends data. Use this e.g. on OpenCPN
const char *ServerIP="192.168.1.100"; // Define the IP, what server will use. This has to be within your local network. Leave empty for DHCP
const size_t MaxClients=10;

bool ResetWiFiSettings=true; // If you have tested other code in your module, it may have saved settings and have difficulties to make connection.

WiFiServer server(ServerPort, MaxClients);

using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

tN2kDataToNMEA0183 tN2kDataToNMEA0183(&NMEA2000, 0);

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {//127489L, // Engine dynamic
                                                  0
                                                 };
const unsigned long ReceiveMessages[] PROGMEM = {/*126992L,*/ // System time
      127250L, // Heading
      127258L, // Magnetic variation
      128259UL,// Boat speed
      128267UL,// Depth
      129025UL,// Position
      129026L, // COG and SOG
      129029L, // GNSS
      130306L, // Wind
      128275L,// Log
      127245L,// Rudder
      129038L,  // AIS Class A Position Report, Message Type 1
      129039L,  // AIS Class B Position Report, Message Type 18
      12979L, // AIS Class A Ship Static and Voyage related data, Message Type 5
      129809L, // AIS Class B "CS" Static Data Report, Part A
      129810L, // AIS Class B "CS" Static Data Report, Part B
      0
    };

// Forward declarations
void LedOn(unsigned long OnTime);
void UpdateLedState();

// Forward declarations NMEA
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg);
void InitNMEA2000();
void SendBufToClients(const char *buf);

// Forward declarations Webserver
void CheckConnections();

#include <nvs.h>
#include <nvs_flash.h>

void ResetWiFiSettingsOnNvs() {
  int err;
  err=nvs_flash_init();
  Serial.println("nvs_flash_init: " + err);
  err=nvs_flash_erase();
  Serial.println("nvs_flash_erase: " + err);
}

//*****************************************************************************
void setup() {
  if ( ResetWiFiSettings ) ResetWiFiSettingsOnNvs();

  pinMode(LED_BUILTIN, OUTPUT);

  // Init USB serial port
  Serial.begin(115200);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.begin(ssid, password);

  size_t WaitCount=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    WaitCount++;
    if ( WaitCount>80 ) {
      Serial.println();
      WaitCount=0;
    }
  }

  if ( ServerIP!=0 && ServerIP[0]!=0 ) { // Try to force ip.
    IPAddress local_IP;
    if ( local_IP.fromString(ServerIP) && !WiFi.config(local_IP,WiFi.gatewayIP(),WiFi.subnetMask()) ) { //, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
    }
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Start TCP server
  server.begin();

  pinMode(GPIO_CAN_DISABLE, INPUT_PULLDOWN);
  delay(1000);
  InitNMEA2000();

}

//*****************************************************************************
void loop() {

  CheckConnections();
  NMEA2000.ParseMessages();
  tN2kDataToNMEA0183.Update();

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) {
    Serial.read();
  }
}

// Reading serial number depends of used board. BoardSerialNumber module
// has methods for RPi, Arduino DUE and Teensy. For others function returns
// 0 and then DefaultSerialNumber will be used.
#define DefaultSerialNumber 999999
//*****************************************************************************
uint32_t GetSerialNumber() {
  uint32_t Sno=GetBoardSerialNumber();
  return ( Sno!=0?Sno:DefaultSerialNumber );
}

//*****************************************************************************
void InitNMEA2000() {
  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(100);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  #if ENABLE_N2K_ON_USB == 1
    NMEA2000.SetForwardStream(&Serial);  // PC output on due native port
  #else
    NMEA2000.EnableForward(false);                 // Disable all msg forwarding to USB (=Serial)
  #endif


  char SnoStr[33];
  uint32_t SerialNumber=GetSerialNumber();
  snprintf(SnoStr,32,"%lu",(long unsigned int)SerialNumber);

  // Set product information
  NMEA2000.SetProductInformation(SnoStr, // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "N2k->NMEA0183 WiFi",  // Manufacturer's Model ID
                                 "2.0.9 (2019-07-07)",  // Manufacturer's Software version code
                                 "1.0.2.0 (2019-07-07)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(SerialNumber, // Unique number. Use e.g. Serial number. Id is generated from MAC-Address
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  #if N2K_TEXTMODE == 1
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  #endif

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 32);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);

  // NMEA 2000 -> NMEA 0183 conversion, as soon as they arrive
  // With this the library calls tN2kDataToNMEA0183::HandleMsg(const tN2kMsg &N2kMsg) on NMEA2000.ParseMessages() when message arrives.
  NMEA2000.AttachMsgHandler(&tN2kDataToNMEA0183);

  tN2kDataToNMEA0183.SetSendNMEA0183MessageCallback(SendNMEA0183Message);

  NMEA2000.Open();
}

#define MAX_NMEA0183_MESSAGE_SIZE 100
//*****************************************************************************
void SendNMEA0183Message(const tNMEA0183Msg &NMEA0183Msg) {
  #if ENABLE_NMEA0183_ON_USB != 1
  return;
  #endif

  char buf[MAX_NMEA0183_MESSAGE_SIZE];
  if ( !NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE) ) return;
  SendBufToClients(buf);
  Serial.println(buf);
}

//***********************  WEBSERVER  *****************************************
// NMEA0183 Sätze an alle Clients senden
void SendBufToClients(const char *buf) {
  for (auto it=clients.begin() ;it!=clients.end(); it++) {
    if ( (*it)!=NULL && (*it)->connected() ) {
      (*it)->println(buf);
    }
  }
}

//*****************************************************************************
void AddClient(WiFiClient &client) {
  Serial.println("New Client.");
  clients.push_back(tWiFiClientPtr(new WiFiClient(client)));
}

//*****************************************************************************
void StopClient(LinkedList<tWiFiClientPtr>::iterator &it) {
  Serial.println("Client Disconnected.");
  (*it)->stop();
  it = clients.erase(it);
}

//*****************************************************************************
void CheckConnections() {
  WiFiClient client = server.available();   // listen for incoming clients

  if ( client ) AddClient(client);

  for (auto it = clients.begin(); it != clients.end(); it++) {
    if ( (*it) != NULL ) {
      if ( !(*it)->connected() ) {
        StopClient(it);
      } else {
        if ( (*it)->available() ) {
          char c = (*it)->read();
          if ( c == 0x03 ) StopClient(it); // Close connection by ctrl-c
        }
      }
    } else {
      it = clients.erase(it); // Should have been erased by StopClient
    }
  }
}
