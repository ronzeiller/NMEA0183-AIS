/*
AISMsg.h

Copyright (c) 2019 Ronnie Zeiller, www.zeiller.eu
Based on the works of Timo Lappalainen NMEA2000 and NMEA0183 Library

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

#ifndef _tAISMsg_H_
#define _tAISMsg_H_

#include <string.h>
#include <time.h>
#include <bitset>
#include <stdint.h>
#include <math.h>
#include <string>


#ifndef AIS_MSG_MAX_LEN
#define AIS_MSG_MAX_LEN 100  // maximum length of AIS Payload
#endif

#ifndef AIS_BIN_MAX_LEN
#define AIS_BIN_MAX_LEN 500  // maximum length of AIS Binary Payload (before encoding to Ascii)
#endif

#define BITSET_LENGTH 120

#ifndef _Time_h
typedef tm tmElements_t;
#endif

class tAISMsg {

	protected:	// AIS-NMEA
		std::bitset<BITSET_LENGTH> bset;
		static const char *EmptyAISField;  // 6bits 0			not used yet.....
		static const char *AsciChar;

		uint16_t iAddPldBin;
    char Payload[AIS_MSG_MAX_LEN];
    uint8_t  iAddPld;

	// Helper functions on converting TimeLib.h to time.h
	protected:	// from NMEAMsg.h
		static unsigned long TimeTDaysTo1970Offset; // Offset for time_t to 1.1.1970. Seem to vary betweel libraries.
	  static unsigned long CalcTimeTDaysTo1970Offset();

	public:
		char PayloadBin[AIS_BIN_MAX_LEN];
		char PayloadBin2[AIS_BIN_MAX_LEN];
		// Clear message
    void Clear();

		public:	// from NMEAMsg.h Timo Lappalainen
			#ifdef _Time_h
			static inline void SetYear(tmElements_t &TimeElements, int val) { TimeElements.Year=val-1970; } //
			static inline void SetMonth(tmElements_t &TimeElements, int val) { TimeElements.Month=val>0?val-1:val; }
			static inline void SetDay(tmElements_t &TimeElements, int val) { TimeElements.Day=val; }
			static inline void SetHour(tmElements_t &TimeElements, int val) { TimeElements.Hour=val; }
			static inline void SetMin(tmElements_t &TimeElements, int val) { TimeElements.Minute=val; }
			static inline void SetSec(tmElements_t &TimeElements, int val) { TimeElements.Second=val; }
			static inline int GetYear(const tmElements_t &TimeElements) { return TimeElements.Year+1970; }
			static inline int GetMonth(const tmElements_t &TimeElements) { return TimeElements.Month+1; }
			static inline int GetDay(const tmElements_t &TimeElements) { return TimeElements.Day; }
			static inline time_t makeTime(tmElements_t &TimeElements) { return ::makeTime(TimeElements); }
			static inline void breakTime(time_t time, tmElements_t &TimeElements) { ::breakTime(time,TimeElements); }
			#else
			static inline void SetYear(tmElements_t &TimeElements, int val) { TimeElements.tm_year=val-1900; } //
			static inline void SetMonth(tmElements_t &TimeElements, int val) { TimeElements.tm_mon=val>0?val-1:val; }
			static inline void SetDay(tmElements_t &TimeElements, int val) { TimeElements.tm_mday=val; }
			static inline void SetHour(tmElements_t &TimeElements, int val) { TimeElements.tm_hour=val; }
			static inline void SetMin(tmElements_t &TimeElements, int val) { TimeElements.tm_min=val; }
			static inline void SetSec(tmElements_t &TimeElements, int val) { TimeElements.tm_sec=val; }
			static inline int GetYear(const tmElements_t &TimeElements) { return TimeElements.tm_year+1900; }
			static inline int GetMonth(const tmElements_t &TimeElements) { return TimeElements.tm_mon+1; }
			static inline int GetDay(const tmElements_t &TimeElements) { return TimeElements.tm_mday; }
			static inline time_t makeTime(tmElements_t &TimeElements) { return mktime(&TimeElements); }
			static inline void breakTime(time_t time, tmElements_t &TimeElements) { TimeElements=*localtime(&time); }
			static time_t daysToTime_t(unsigned long val);
			#endif
			static unsigned long elapsedDaysSince1970(time_t dt);


	public:
		tAISMsg();
		const char *GetPayload();
		const char *GetPayloadType5_Part1();
		const char *GetPayloadType5_Part2();
		const char *GetPayloadType24_PartA();
		const char *GetPayloadType24_PartB();
		const char *GetPayloadBin() const { return  PayloadBin; }

		// Generally Used
		bool AddIntToPayloadBin(int32_t ival, uint16_t countBits);
		bool AddBool(bool &bval, uint8_t size);
		bool AddEncodedCharToAscii(char *sval, size_t Length);

	// AIS Helper functions
	protected:
		inline int32_t aRoundToInt(double x) {
		  return (x >= 0) ? (int32_t) floor(x + 0.5) : (int32_t) ceil(x - 0.5);
		}

		bool convertBinaryAISPayloadBinToAscii(const char *payloadbin);

		bool AddAISEmptyField(uint8_t iBits);

};
#endif
