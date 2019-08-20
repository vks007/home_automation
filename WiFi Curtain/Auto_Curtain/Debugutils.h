/*
 *DebugUtils.h - Simple debugging utilities. Include the file "Debugutils.h in your program.
 * To turn DEBUG ON & OFF put the following statement in your program
 #define BEDUG //this statement should be before the include statement for Debugutils.h
 #include "Debugutils.h"
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

#endif
