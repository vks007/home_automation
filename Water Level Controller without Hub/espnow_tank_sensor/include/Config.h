#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

#define WATER_TANK_SENSOR 1

#if (DEVICE == WATER_TANK_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "water_tank_sensor" //max 15 characters without spaces
  #define SENSOR_PIN              4 //supplies power to the temp sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  uint8_t gatewayAddress[] =      GATEWAY_TEST_AP_MAC;//ESP32_DEVKIT_AP_MAC; //comes from secrets.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
