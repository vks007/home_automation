#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

#define WATER_TANK_SENSOR 1

#if (DEVICE == WATER_TANK_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "water_tank_sensor" //max 15 characters without spaces
  #define SENSOR_PIN              4 // GPIO pin on which the float sensor is connected
  uint8_t gatewayAddress[] =      BROADCAST_AP_MAC;//CONTROLLER_W8_AP_MAC; // GATEWAY_FF_AP_MAC;// //comes from secrets.h
  // #define WiFi_SSID               primary_ssid //from secrets.h
  // #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define DEFAULT_CHANNEL         1
  #define SLEEP_DURATION          5 // sleep time interval in seconds
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
