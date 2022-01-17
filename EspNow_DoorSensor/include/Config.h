#ifndef CONFIG_H
#define CONFIG_H

// you cant use strings so you have to use #defines for numeric values
// Ref :https://stackoverflow.com/questions/2335888/how-to-compare-strings-in-c-conditional-preprocessor-directives
#define USE_MAIN_DOOR 1
#define USE_TERRACE_DOOR 2
#define USE_BALCONY_DOOR 3

#if (DEVICE == USE_MAIN_DOOR) 
  #pragma message "Compiling the program for the device: MAIN_DOOR"
  #define DEVICE_NAME             "main_door"
  //Hold pin will hold CH_PD HIGH till we're executing the setup, the last step would be set it LOW which will power down the ESP
  #define HOLD_PIN 0  // defines GPIO0 as the hold pin (will hold CH_PD high untill we power down).
  #define SIGNAL_PIN0 1 //Bit 1 of the signal which indicates the message type
  #define SIGNAL_PIN1 3 //Bit 2 of the signal which indicates the message type
  //State Mapping of SIGNAL_PIN0 SIGNAL_PIN1:: 11=>IDLE , 00=> SENSOR_WAKEUP , 01=> SENSOR OPEN , 10=> SENSOR CLOSED
  uint8_t gatewayAddress[] = GATEWAY_GF_MAC; //comes from secrets.h

#elif (DEVICE == USE_TERRACE_DOOR)
  #pragma message "Compiling the program for the device: TERRACE_DOOR"
  #define DEVICE_NAME             "terrace_door" // This becomes the postfix of the final MQTT topic under which messages are published
  //Hold pin will hold CH_PD HIGH till we're executing the setup, the last step would be set it LOW which will power down the ESP
  #define HOLD_PIN 0  // defines GPIO0 as the hold pin (will hold CH_PD high untill we power down).
  #define SIGNAL_PIN0 1 //Bit 1 of the signal which indicates the message type
  #define SIGNAL_PIN1 3 //Bit 2 of the signal which indicates the message type
  //State Mapping of SIGNAL_PIN0 SIGNAL_PIN1:: 11=>IDLE , 00=> SENSOR_WAKEUP , 01=> SENSOR OPEN , 10=> SENSOR CLOSED
  uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h

#elif (DEVICE == USE_BALCONY_DOOR)
  #pragma message "Compiling the program for the device: BALCONY_DOOR"
  #define DEVICE_NAME             "balcony_door"
  #define HOLD_PIN 0  // defines GPIO0 as the hold pin (will hold CH_PD high untill we power down).
  #define SIGNAL_PIN0 5 //Bit 1 of the signal which indicates the message type
  #define SIGNAL_PIN1 4 //Bit 2 of the signal which indicates the message type
  uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h

#else
  #error "Device type not selected, see Door_config.h"
#endif

#endif
