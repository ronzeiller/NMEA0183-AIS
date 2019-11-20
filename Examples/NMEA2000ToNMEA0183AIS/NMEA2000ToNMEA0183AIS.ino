/*
NMEA2000ToNMEA0183AIS

NMEA0183/NMEA2000 library. NMEA2000 -> NMEA0183 including AIS TYPE 1, CLASS A

 NMEA2000ToNMEA0183 Example taken from Timo Lappalainen
 Extended for AIS Encoding by Ronnie Zeiller
 
   Reads some messages from NMEA2000 and converts them to NMEA0183
   format to NMEA0183_out (Serial on Arduino or /dev/tnt0 on RPi).
   Also forwards all NMEA2000 bus messages in Actisense format.

   The example is designed for sending N2k data to OpenCPN on RPi with PiCAN2.
   It can be used also on PC with some Arduino/Teensy board to provide
   NMEA0183 (and also NMEA2000) data to PC. Example has been tested on
   RPi3B, ESP32, Arduino DUE, Arduino Mega and Teensy.

 To use this example you need install also:
   - NMEA2000 library
   - NMEA0183 library
   - Related CAN libraries.

 The example works with ESP32 MCU Kit from AZ-Delivery which should be a 1:1 clone from Espressif 32 Kit
*/

// This file is just Arduino IDE wrapper.
