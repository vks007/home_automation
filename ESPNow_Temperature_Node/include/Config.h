#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

#define GENERIC_SENSOR 1
#define SOLAR_GEYSER_SENSOR 2

#if (DEVICE == GENERIC_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not
  #define TESTING_MODE            IN_USE // defines if we're in testing mode in which the sensor keeps sending readings in a loop for MAX_COUNT
  #define MY_ROLE                 ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "generic_sensor" //max 15 characters without spaces
  #define SENSOR_POWER_PIN        5 //supplies power to the temp sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  #define TEMPERATURE_SENSOR_PIN  4 // gets readings from the data pin of DS18B20 sensor , there should be a pull up from this pin to Vcc
  #define RESISTOR_CONST          5.156 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R2 , // I have used R1 = 1M , R2=270K. calc factor comes to 4.7 but actual measurements gave me a more precise value of 5.156
  uint8_t gatewayAddress[] =      GATEWAY_TEST_AP_MAC; //GATEWAY_TEST_AP_MAC;//ESP32_DEVKIT_AP_MAC; //comes from secrets.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define MIN_TEMP                0 // minimum temprature which you want to measure, this would be the lowest value it will send
  #define DEFAULT_CHANNEL         11
  #define SLEEP_DURATION          10 // sleep time interval in seconds , see notes on its usage, this isnt working well
#elif (DEVICE == SOLAR_GEYSER_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not
  #define TESTING_MODE            NOT_IN_USE // defines if we're in testing mode in which the sensor keeps sending readings in a loop for MAX_COUNT
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "solar_geyser" //no spaces as this is used in topic names too
  #define SENSOR_POWER_PIN        5 //supplies power to the temp sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  #define TEMPERATURE_SENSOR_PIN  4 // gets readings from the data pin of DS18B20 sensor , there should be a pull up from this pin to Vcc
  #define RESISTOR_CONST          5.156 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R2 , // I have used R1 = 1M , R2=270K. calc factor comes to 4.7 but actual measurements gave me a more precise value of 5.156
  uint8_t gatewayAddress[] =      GATEWAY_SF_AP_MAC; //comes from secrets.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define MIN_TEMP                0 // minimum temprature which you want to measure, this would be the lowest value it will send and also send it when it has invalid readings
  #define DEFAULT_CHANNEL         11
  #define SLEEP_DURATION          60 // sleep time interval in seconds , see notes on its usage, this isnt working well
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
