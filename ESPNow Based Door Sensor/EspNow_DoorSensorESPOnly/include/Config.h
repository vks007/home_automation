#ifndef CONFIG_H
#define CONFIG_H

// you cant use strings so you have to use #defines for numeric values
// Ref :https://stackoverflow.com/questions/2335888/how-to-compare-strings-in-c-conditional-preprocessor-directives
//The following defines the device type for which the program is being compiled, this is passed as a compile time parameter in platform.ini
#define DOOR1 1
#define DOOR2 2

#if (DEVICE == DOOR1) 
  #pragma message "Compiling the program for the device: DOOR1"
  #define DEVICE_NAME             "door1"
  #define SIGNAL_PIN 3 //pin used to indicate the sensor as open or closed 
  //State Mapping of SIGNAL_PIN :: 1=> SENSOR OPEN , 0=> SENSOR CLOSED
  #define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver
  uint8_t gatewayAddress[] = GATEWAY_GF_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] = gf_ssid;// from secrets.h,ssid to which the slave(gateway) connects, it is used only get the channel number, password for ssid isnt required

#elif (DEVICE == DOOR2)
  #pragma message "Compiling the program for the device: DOOR2"
  #define DEVICE_NAME             "door2" // This becomes the postfix of the final MQTT topic under which messages are published
  #define SIGNAL_PIN 3 //pin used to indicate the sensor as open or closed 
  //State Mapping of SIGNAL_PIN :: 0=> SENSOR OPEN , 1=> SENSOR CLOSED
  #define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver
  // gateway MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
  // You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address
  uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] = primary_ssid;// from secrets.h,ssid to which the slave(gateway) connects, it is used only get the channel number, password for ssid isnt required

#else
  #error "Device type not selected, see Door_config.h"
#endif

#endif
