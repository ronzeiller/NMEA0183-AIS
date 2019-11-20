= NMEA2000 -> NMEA0183 AIS converter  v1.0.0 =


NMEA0183  AIS library © Ronnie Zeiller, www.zeiller.eu

Addendum for NMEA2000 and NMEA0183 Library from Timo Lappalainen https://github.com/ttlappalainen


== Conversions: ==

NMEA2000 PGN 129038 => AIS CLASS A Position Report (Message Type 1) a.) b.) c.)
NMEA2000 PGN 129039 => AIS Class B Position Report, Message Type 18
NMEA2000 PGN 129794 => AIS Class A Ship Static and Voyage related data, Message Type 5 d.)
NMEA2000 PGN 129809 => AIS Class B "CS" Static Data Report, making a list of UserID (MMSI) and Ship Names used for Message 24 Part A
NMEA2000 PGN 129810 => AIS Class B "CS" Static Data Report, Message 24 Part A+B

=== Remarks ===
a.) Message Type could be set to 1 or 3 (identical messages) on demand
b.) Maneuver Indicator (not part of NMEA2000 PGN 129038) => will be set to 0 (default)
c.) Radio Status (not part of NMEA2000 PGN 129038) => will be set to 0
d.) AIS Version (not part of NMEA2000 PGN 129794) => will be set to 1

== Dependencies ==

To use this library you need also:

   - NMEA2000 library

   - NMEA0183 library

   - Related CAN libraries.
