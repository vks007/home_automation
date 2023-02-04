/*
 *DebugUtils.h - Simple debugging macros for easy management of Debug code for Arduino/ESP.
 * To turn DEBUG ON & OFF put the following statement in your program , this statement should be before the include statement for Debugutils.h
 #define SERIAL_DEBUG           IN_USE // Turn DEBUG ON
 #define SERIAL_DEBUG           NOT_IN_USE // Turn DEBUG OFF
 * the file defines two ways to define Serial, one if the standard Serial using default hardware serial ports , for this use SERIAL_DEBUG
 * If you want to use SoftwareSerial library on some other pins then you can use SERIAL1_DEBUG instead and pass the pin numbers used for SoftwareSerial
 #include "Debugutils.h"
*/

#pragma once

#include "macros.h"

#if defined(SERIAL_DEBUG)
  #if USING(SERIAL_DEBUG)
    #define DPRINT(...) Serial.print(__VA_ARGS__)
    #define DPRINTLN(...) Serial.println(__VA_ARGS__)
    #define DBEGIN(...) Serial.begin(__VA_ARGS__)
    #define DPRINTF(...) Serial.printf(__VA_ARGS__)
    #define DPRINTFLN(...) Serial.printf(__VA_ARGS__);Serial.printf("\n")
    #define DFLUSH()		Serial.flush()
    #define DEND()		Serial.end()
  #else
    #define DPRINT(...)
    #define DPRINTLN(...)
    #define DBEGIN(...)
    #define DPRINTF(...)
    #define DPRINTFLN(...)
    #define DFLUSH()
    #define DEND()
  #endif
#elif defined(WEBSERIAL_DEBUG)
  #if USING(WEBSERIAL_DEBUG)
    #define DPRINT(...) Serial.print(__VA_ARGS__);WebSerial.print(__VA_ARGS__);
    #define DPRINTLN(...) Serial.println(__VA_ARGS__);WebSerial.println(__VA_ARGS__);
    #define DBEGIN(...) Serial.begin(__VA_ARGS__)
    #define DFLUSH()		Serial.flush()
    #define DEND()		Serial.end()
	#define WBEGIN(...) WebSerial.begin(__VA_ARGS__)
    #define DPRINTF(...) Serial.printf(__VA_ARGS__);WebSerial.printf(__VA_ARGS__)
    #define DPRINTFLN(...) Serial.printf(__VA_ARGS__);Serial.println("");WebSerial.printf(__VA_ARGS__);WebSerial.println("")
  #else
    #define DPRINT(...)
    #define DPRINTLN(...)
    #define DBEGIN(...)
    #define DPRINTF(...)
    #define DPRINTFLN(...)
    #define DFLUSH()
    #define DEND()
  #endif
#elif defined(WEBSERIAL)
  #if USING(WEBSERIAL)
    #define DPRINT(...) Serial.print(__VA_ARGS__);WebSerial.print(__VA_ARGS__);
    #define DPRINTLN(...) Serial.println(__VA_ARGS__);WebSerial.println(__VA_ARGS__);
    #define DBEGIN(...) Serial.begin(__VA_ARGS__)
    #define DFLUSH()		Serial.flush()
    #define DEND()		Serial.end()
	#define WBEGIN(...) WebSerial.begin(__VA_ARGS__)
    #define DPRINTF(...) Serial.printf(__VA_ARGS__);WebSerial.printf(__VA_ARGS__)
    #define DPRINTFLN(...) Serial.printf(__VA_ARGS__);Serial.println("");WebSerial.printf(__VA_ARGS__);WebSerial.println("")
  #else
    #define DPRINT(...)
    #define DPRINTLN(...)
    #define DBEGIN(...)
    #define DPRINTF(...)
    #define DPRINTFLN(...)
    #define DFLUSH()
    #define DEND()
  #endif
#elif defined(SERIAL1_DEBUG)
  #if USING(SERIAL1_DEBUG)
    #define DPRINT(...) Serial1.print(__VA_ARGS__)
    #define DPRINTLN(...) Serial1.println(__VA_ARGS__)
    #define DBEGIN(...) Serial1.begin(__VA_ARGS__)
    #define DPRINTF(...) Serial1.printf(__VA_ARGS__)
    #define DPRINTFLN(...) Serial1.printf(__VA_ARGS__);Serial.printf("\n")
    #define DFLUSH()		Serial1.flush()
    #define DEND()		Serial1.end()
  #else
    #define DPRINT(...)
    #define DPRINTLN(...)
    #define DBEGIN(...)
    #define DPRINTF(...)
    #define DPRINTFLN(...)
    #define DFLUSH()
    #define DEND()
  #endif
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
  #define DBEGIN(...)
  #define DPRINTF(...)
  #define DPRINTFLN(...)
  #define DFLUSH()
  #define DEND()
#endif

//#define PRINTFEATURE(name,feature) Serial.print(name);Serial.print("-ON");
// #if USING(feature)
//   Serial.print(name);Serial.print("-ON");
// #else
//   DPRINT(name);DPRINTLN("-OFF");
// #endif


