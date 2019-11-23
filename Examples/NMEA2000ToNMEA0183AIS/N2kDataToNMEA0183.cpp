/*
N2kDataToNMEA0183.cpp

Copyright (c) 2015-2018 Timo Lappalainen, Kave Oy, www.kave.fi
Addon for AIS (c) 2019 Ronnie Zeiller, www.zeiller.eu

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "N2kDataToNMEA0183.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include <NMEA0183AISMessages.h>
#include <NMEA0183AISMsg.h>

#define SERIAL_PRINT_AIS_NMEA    // Prints the !AIVDM NMEA sentence on Serial
//#define SERIAL_PRINT_AIS_FIELDS  // Prints parsed datas derived from N2kParser

//*****************************************************************************
void tN2kDataToNMEA0183::HandleMsg(const tN2kMsg &N2kMsg) {
  switch (N2kMsg.PGN) {
    case 127250UL: HandleHeading(N2kMsg); break;
    case 127258UL: HandleVariation(N2kMsg); break;
    case 128259UL: HandleBoatSpeed(N2kMsg); break;
    case 128267UL: HandleDepth(N2kMsg); break;
    case 129025UL: HandlePosition(N2kMsg); break;
    case 129026UL: HandleCOGSOG(N2kMsg); break;
    case 129029UL: HandleGNSS(N2kMsg); break;
    // AIS
    case 129038UL: HandleAISClassAPosReport(N2kMsg); break;   // AIS Class A Position Report, Message Type 1
    case 129039UL: HandleAISClassBMessage18(N2kMsg); break;   // AIS Class B Position Report, Message Type 18
    case 129794UL: HandleAISClassAMessage5(N2kMsg);  break;   // AIS Class A Ship Static and Voyage related data, Message Type 5
    case 129809UL: HandleAISClassBMessage24A(N2kMsg); break;  // AIS Class B "CS" Static Data Report, Part A
    case 129810UL: HandleAISClassBMessage24B(N2kMsg); break;  // AIS Class B "CS" Static Data Report, Part B
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::Update() {
  SendRMC();
  if ( LastHeadingTime+2000<millis() ) Heading=N2kDoubleNA;
  if ( LastCOGSOGTime+2000<millis() ) { COG=N2kDoubleNA; SOG=N2kDoubleNA; }
  if ( LastPositionTime+4000<millis() ) { Latitude=N2kDoubleNA; Longitude=N2kDoubleNA; }
}

//*****************************************************************************
void tN2kDataToNMEA0183::SendMessage(const tNMEA0183Msg &NMEA0183Msg) {
  if ( pNMEA0183!=0 ) pNMEA0183->SendMessage(NMEA0183Msg);
  if ( SendNMEA0183MessageCallback!=0 ) SendNMEA0183MessageCallback(NMEA0183Msg);
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleHeading(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference ref;
double _Deviation=0;
double _Variation;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kHeading(N2kMsg, SID, Heading, _Deviation, _Variation, ref) ) {
    if ( ref==N2khr_magnetic ) {
      if ( !N2kIsNA(_Variation) ) Variation=_Variation; // Update Variation
      if ( !N2kIsNA(Heading) && !N2kIsNA(Variation) ) Heading-=Variation;
    }
    LastHeadingTime=millis();
    if ( NMEA0183SetHDG(NMEA0183Msg,Heading,_Deviation,Variation) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleVariation(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kMagneticVariation Source;

  ParseN2kMagneticVariation(N2kMsg,SID,Source,DaysSince1970,Variation);
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleBoatSpeed(const tN2kMsg &N2kMsg) {
unsigned char SID;
double WaterReferenced;
double GroundReferenced;
tN2kSpeedWaterReferenceType SWRT;

  if ( ParseN2kBoatSpeed(N2kMsg,SID,WaterReferenced,GroundReferenced,SWRT) ) {
    tNMEA0183Msg NMEA0183Msg;
    double MagneticHeading=( !N2kIsNA(Heading) && !N2kIsNA(Variation)?Heading+Variation: NMEA0183DoubleNA);
    if ( NMEA0183SetVHW(NMEA0183Msg,Heading,MagneticHeading,WaterReferenced) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleDepth(const tN2kMsg &N2kMsg) {
unsigned char SID;
double DepthBelowTransducer;
double Offset;
double Range;

  if ( ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset,Range) ) {
      tNMEA0183Msg NMEA0183Msg;
      if ( NMEA0183SetDPT(NMEA0183Msg,DepthBelowTransducer,Offset) ) {
        SendMessage(NMEA0183Msg);
      }
      if ( NMEA0183SetDBx(NMEA0183Msg,DepthBelowTransducer,Offset) ) {
        SendMessage(NMEA0183Msg);
      }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandlePosition(const tN2kMsg &N2kMsg) {

  if ( ParseN2kPGN129025(N2kMsg, Latitude, Longitude) ) {
    LastPositionTime=millis();
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleCOGSOG(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference HeadingReference;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
    LastCOGSOGTime=millis();
    double MCOG=( !N2kIsNA(COG) && !N2kIsNA(Variation)?COG-Variation:NMEA0183DoubleNA );
    if ( HeadingReference==N2khr_magnetic ) {
      MCOG=COG;
      if ( !N2kIsNA(Variation) ) COG-=Variation;
    }
    if ( NMEA0183SetVTG(NMEA0183Msg,COG,MCOG,SOG) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleGNSS(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kGNSStype GNSStype;
tN2kGNSSmethod GNSSmethod;
unsigned char nSatellites;
double HDOP;
double PDOP;
double GeoidalSeparation;
unsigned char nReferenceStations;
tN2kGNSStype ReferenceStationType;
uint16_t ReferenceSationID;
double AgeOfCorrection;

  if ( ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,Latitude,Longitude,Altitude,GNSStype,GNSSmethod,
                    nSatellites,HDOP,PDOP,GeoidalSeparation,
                    nReferenceStations,ReferenceStationType,ReferenceSationID,AgeOfCorrection) ) {
    LastPositionTime=millis();
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::SendRMC() {
    if ( NextRMCSend<=millis() && !N2kIsNA(Latitude) ) {
      tNMEA0183Msg NMEA0183Msg;
      if ( NMEA0183SetRMC(NMEA0183Msg,SecondsSinceMidnight,Latitude,Longitude,COG,SOG,DaysSince1970,Variation) ) {
        SendMessage(NMEA0183Msg);
      }
      SetNextRMCSend();
    }
}

//*****************************************************************************
// 129038 AIS Class A Position Report (Message 1, 2, 3)
void tN2kDataToNMEA0183::HandleAISClassAPosReport(const tN2kMsg &N2kMsg) {

  unsigned char SID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;  // MMSI
  double _Latitude;
  double _Longitude;
  bool _Accuracy;
  bool _RAIM;
  uint8_t _Seconds;
  double _COG;
  double _SOG;
  double _Heading;
  double _ROT;
  tN2kAISNavStatus _NavStatus;

  uint8_t _MessageType = 1;
  tNMEA0183AISMsg NMEA0183AISMsg;

  if ( ParseN2kPGN129038(N2kMsg, SID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM, _Seconds,
                          _COG, _SOG, _Heading, _ROT, _NavStatus ) ) {

    // Debug
    #ifdef SERIAL_PRINT_AIS_FIELDS
      Serial.println("–––––––––––––––––––––––– Msg 1 ––––––––––––––––––––––––––––––––");

      const double pi=3.1415926535897932384626433832795;
      const double radToDeg=180.0/pi;
      const double msTokn=3600.0/1852.0;
      const double radsToDegMin = 60 * 360.0 / (2 * pi);    // [rad/s -> degree/minute]
      Serial.print("Repeat: "); Serial.println(_Repeat);
      Serial.print("UserID: "); Serial.println(_UserID);
      Serial.print("Latitude: "); Serial.println(_Latitude);
      Serial.print("Longitude: "); Serial.println(_Longitude);
      Serial.print("Accuracy: "); Serial.println(_Accuracy);
      Serial.print("RAIM: "); Serial.println(_RAIM);
      Serial.print("Seconds: "); Serial.println(_Seconds);
      Serial.print("COG: "); Serial.println(_COG*radToDeg);
      Serial.print("SOG: "); Serial.println(_SOG*msTokn);
      Serial.print("Heading: "); Serial.println(_Heading*radToDeg);
      Serial.print("ROT: "); Serial.println(_ROT*radsToDegMin);
      Serial.print("NavStatus: "); Serial.println(_NavStatus);
    #endif

    if ( SetAISClassABMessage1(NMEA0183AISMsg, _MessageType, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy,
                          _RAIM, _Seconds, _COG, _SOG, _Heading, _ROT, _NavStatus ) ) {

      SendMessage(NMEA0183AISMsg);

      #ifdef SERIAL_PRINT_AIS_NMEA
        // Debug Print AIS-NMEA
        Serial.print(NMEA0183AISMsg.GetPrefix());
        Serial.print(NMEA0183AISMsg.Sender());
        Serial.print(NMEA0183AISMsg.MessageCode());
        for (int i=0; i<NMEA0183AISMsg.FieldCount(); i++) {
          Serial.print(",");
          Serial.print(NMEA0183AISMsg.Field(i));
        }
        char buf[7];
        sprintf(buf,"*%02X\r\n",NMEA0183AISMsg.GetCheckSum());
        Serial.print(buf);
      #endif
    }
  }
}  // end 129038 AIS Class A Position Report Message 1/3

//*****************************************************************************
// 129039 AIS Class B Position Report -> AIS Message Type 5: Static and Voyage Related Data
void tN2kDataToNMEA0183::HandleAISClassAMessage5(const tN2kMsg &N2kMsg) {
  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;  // MMSI
  uint32_t _IMONumber;
  char _Callsign[8];
  char _Name[21];
  uint8_t _VesselType;
  double _Length;
  double _Beam;
  double _PosRefStbd;
  double _PosRefBow;
  uint16_t _ETAdate;
  double _ETAtime;
  double _Draught;
  char _Destination[21];
  tN2kAISVersion _AISversion;
  tN2kGNSStype _GNSStype;
  tN2kAISTranceiverInfo _AISinfo;
  tN2kAISDTE _DTE;

  tNMEA0183AISMsg NMEA0183AISMsg;

  if ( ParseN2kPGN129794(N2kMsg, _MessageID, _Repeat, _UserID, _IMONumber, _Callsign, _Name, _VesselType,
                        _Length, _Beam, _PosRefStbd, _PosRefBow, _ETAdate, _ETAtime, _Draught, _Destination,
                        _AISversion, _GNSStype, _DTE, _AISinfo) ) {

    #ifdef SERIAL_PRINT_AIS_FIELDS
      // Debug Print N2k Values
      Serial.println("––––––––––––––––––––––– Msg 5 –––––––––––––––––––––––––––––––––");
      Serial.print("MessageID: "); Serial.println(_MessageID);
      Serial.print("Repeat: "); Serial.println(_Repeat);
      Serial.print("UserID: "); Serial.println(_UserID);
      Serial.print("IMONumber: "); Serial.println(_IMONumber);
      Serial.print("Callsign: "); Serial.println(_Callsign);
      Serial.print("VesselType: "); Serial.println(_VesselType);
      Serial.print("Name: "); Serial.println(_Name);
      Serial.print("Length: "); Serial.println(_Length);
      Serial.print("Beam: "); Serial.println(_Beam);
      Serial.print("PosRefStbd: "); Serial.println(_PosRefStbd);
      Serial.print("PosRefBow: "); Serial.println(_PosRefBow);
      Serial.print("ETAdate: "); Serial.println(_ETAdate);
      Serial.print("ETAtime: "); Serial.println(_ETAtime);
      Serial.print("Draught: "); Serial.println(_Draught);
      Serial.print("Destination: "); Serial.println(_Destination);
      Serial.print("GNSStype: "); Serial.println(_GNSStype);
      Serial.print("DTE: "); Serial.println(_DTE);
      Serial.println("––––––––––––––––––––––– Msg 5 –––––––––––––––––––––––––––––––––");
    #endif

    if ( SetAISClassAMessage5(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _IMONumber, _Callsign, _Name, _VesselType,
                              _Length, _Beam, _PosRefStbd, _PosRefBow, _ETAdate, _ETAtime, _Draught, _Destination,
                              _GNSStype, _DTE ) ) {

      SendMessage( NMEA0183AISMsg.BuildMsg5Part1(NMEA0183AISMsg) );

      #ifdef SERIAL_PRINT_AIS_NMEA
      // Debug Print AIS-NMEA Message Type 5, Part 1
      char buf[7];
      Serial.print(NMEA0183AISMsg.GetPrefix());
      Serial.print(NMEA0183AISMsg.Sender());
      Serial.print(NMEA0183AISMsg.MessageCode());
      for (int i=0; i<NMEA0183AISMsg.FieldCount(); i++) {
        Serial.print(",");
        Serial.print(NMEA0183AISMsg.Field(i));
      }
      sprintf(buf,"*%02X\r\n",NMEA0183AISMsg.GetCheckSum());
      Serial.print(buf);
      #endif

      SendMessage( NMEA0183AISMsg.BuildMsg5Part2(NMEA0183AISMsg) );

      #ifdef SERIAL_PRINT_AIS_NMEA
      // Print AIS-NMEA Message Type 5, Part 2
      Serial.print(NMEA0183AISMsg.GetPrefix());
      Serial.print(NMEA0183AISMsg.Sender());
      Serial.print(NMEA0183AISMsg.MessageCode());
      for (int i=0; i<NMEA0183AISMsg.FieldCount(); i++) {
        Serial.print(",");
        Serial.print(NMEA0183AISMsg.Field(i));
      }
      sprintf(buf,"*%02X\r\n",NMEA0183AISMsg.GetCheckSum());
      Serial.print(buf);
      #endif
    }
  }
}

//
//*****************************************************************************
// 129039 AIS Class B Position Report (Message 18)
void tN2kDataToNMEA0183::HandleAISClassBMessage18(const tN2kMsg &N2kMsg) {
  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;  // MMSI
  double _Latitude;
  double _Longitude;
  bool _Accuracy;
  bool _RAIM;
  uint8_t _Seconds;
  double _COG;
  double _SOG;
  double _Heading;
  tN2kAISUnit _Unit;
  bool _Display, _DSC, _Band, _Msg22, _State;
  tN2kAISMode _Mode;

  if ( ParseN2kPGN129039(N2kMsg, _MessageID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM,
                     _Seconds, _COG, _SOG, _Heading, _Unit, _Display, _DSC, _Band, _Msg22, _Mode, _State) ) {

    tNMEA0183AISMsg NMEA0183AISMsg;

    if ( SetAISClassBMessage18(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM,
                     _Seconds, _COG, _SOG, _Heading, _Unit, _Display, _DSC, _Band, _Msg22, _Mode, _State) ) {

      SendMessage(NMEA0183AISMsg);

      #ifdef SERIAL_PRINT_AIS_NMEA
      // Debug Print AIS-NMEA
      Serial.print(NMEA0183AISMsg.GetPrefix());
      Serial.print(NMEA0183AISMsg.Sender());
      Serial.print(NMEA0183AISMsg.MessageCode());
      for (int i=0; i<NMEA0183AISMsg.FieldCount(); i++) {
        Serial.print(",");
        Serial.print(NMEA0183AISMsg.Field(i));
      }
      char buf[7];
      sprintf(buf,"*%02X\r\n",NMEA0183AISMsg.GetCheckSum());
      Serial.print(buf);
      #endif
    }
  }
  return;
}

//*****************************************************************************
// PGN 129809 AIS Class B "CS" Static Data Report, Part A
void tN2kDataToNMEA0183::HandleAISClassBMessage24A(const tN2kMsg &N2kMsg) {

  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;  // MMSI
  char _Name[21];

  if ( ParseN2kPGN129809 (N2kMsg, _MessageID, _Repeat, _UserID, _Name) ) {

    tNMEA0183AISMsg NMEA0183AISMsg;
    if ( SetAISClassBMessage24PartA(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _Name) ) {}
  }
  return;
}

//*****************************************************************************
// PGN 129810 AIS Class B "CS" Static Data Report, Part B -> AIS Message 24 (2 Parts)
void tN2kDataToNMEA0183::HandleAISClassBMessage24B(const tN2kMsg &N2kMsg) {

  uint8_t _MessageID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID, _MothershipID;  // MMSI
  char _Callsign[8];
  char _Vendor[4];
  uint8_t _VesselType;
  double _Length;
  double _Beam;
  double _PosRefStbd;
  double _PosRefBow;

  if ( ParseN2kPGN129810(N2kMsg, _MessageID, _Repeat, _UserID, _VesselType, _Vendor, _Callsign,
                        _Length, _Beam, _PosRefStbd, _PosRefBow, _MothershipID) ) {

    //
    #ifdef SERIAL_PRINT_AIS_FIELDS
    // Debug Print N2k Values
    Serial.println("––––––––––––––––––––––– Msg 24 ––––––––––––––––––––––––––––––––");
    Serial.print("MessageID: "); Serial.println(_MessageID);
    Serial.print("Repeat: "); Serial.println(_Repeat);
    Serial.print("UserID: "); Serial.println(_UserID);
    Serial.print("VesselType: "); Serial.println(_VesselType);
    Serial.print("Vendor: "); Serial.println(_Vendor);
    Serial.print("Callsign: "); Serial.println(_Callsign);
    Serial.print("Length: "); Serial.println(_Length);
    Serial.print("Beam: "); Serial.println(_Beam);
    Serial.print("PosRefStbd: "); Serial.println(_PosRefStbd);
    Serial.print("PosRefBow: "); Serial.println(_PosRefBow);
    Serial.print("MothershipID: "); Serial.println(_MothershipID);
    Serial.println("––––––––––––––––––––––– Msg 24 ––––––––––––––––––––––––––––––––");
    #endif

    tNMEA0183AISMsg NMEA0183AISMsg;

    if ( SetAISClassBMessage24(NMEA0183AISMsg, _MessageID, _Repeat, _UserID, _VesselType, _Vendor, _Callsign,
                          _Length, _Beam, _PosRefStbd, _PosRefBow, _MothershipID ) ) {

      SendMessage( NMEA0183AISMsg.BuildMsg24PartA(NMEA0183AISMsg) );

      #ifdef SERIAL_PRINT_AIS_NMEA
      // Debug Print AIS-NMEA
      char buf[7];
      Serial.print(NMEA0183AISMsg.GetPrefix());
      Serial.print(NMEA0183AISMsg.Sender());
      Serial.print(NMEA0183AISMsg.MessageCode());
      for (int i=0; i<NMEA0183AISMsg.FieldCount(); i++) {
        Serial.print(",");
        Serial.print(NMEA0183AISMsg.Field(i));
      }
      sprintf(buf,"*%02X\r\n",NMEA0183AISMsg.GetCheckSum());
      Serial.print(buf);
      #endif

      SendMessage( NMEA0183AISMsg.BuildMsg24PartB(NMEA0183AISMsg) );

      #ifdef SERIAL_PRINT_AIS_NMEA
      Serial.print(NMEA0183AISMsg.GetPrefix());
      Serial.print(NMEA0183AISMsg.Sender());
      Serial.print(NMEA0183AISMsg.MessageCode());
      for (int i=0; i<NMEA0183AISMsg.FieldCount(); i++) {
        Serial.print(",");
        Serial.print(NMEA0183AISMsg.Field(i));
      }
      sprintf(buf,"*%02X\r\n",NMEA0183AISMsg.GetCheckSum());
      Serial.print(buf);
      #endif
    }
  }
  return;
}
