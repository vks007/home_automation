#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

#define W8_TANK_SENSOR 1
#define TEST_TANK_SENSOR 2
#define X15_TANK_SENSOR 3

#if (DEVICE == W8_TANK_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "w8_tank_sensor" //max 15 characters without spaces
  #define SENSOR_PIN              4 // GPIO pin on which the float sensor is connected
  #define WATER_PRESENCE_SENSOR_PIN 12 // GPIO pin on which the water presence sensor is connected
  uint8_t gatewayAddress[] =      BROADCAST_AP_MAC; //CONTROLLER_W8_AP_MAC;  //comes from secrets.h
  #define WiFi_SSID               w8_ssid //from secrets.h
  // #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define DEFAULT_CHANNEL         1
  #define SLEEP_DURATION          25 // sleep time interval in seconds
  #define BATT_RESISTOR_CONST     4.59 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R2
        // I have used R1 = 968K , R2=265K. calc factor comes to 4.61 but actual measurements gave me a more precise value of 4.59
#elif (DEVICE == TEST_TANK_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "test_sensor" //max 15 characters without spaces
  #define SENSOR_PIN              13 // GPIO pin on which the float sensor is connected
  #define WATER_PRESENCE_SENSOR_PIN 12 // GPIO pin on which the water presence sensor is connected
  uint8_t gatewayAddress[] =      GATEWAY_SF_AP_MAC; //comes from secrets.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  // #define WiFi_SSID_PSWD       primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define DEFAULT_CHANNEL         1
  #define SLEEP_DURATION          10 // sleep time interval in seconds
  #define BATT_RESISTOR_CONST     5.3773 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R2
        // I have used R1 = 968K , R2=265K. calc factor comes to 4.61 but actual measurements gave me a more precise value of 4.59
#elif (DEVICE == X15_TANK_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "x15_tank_sensor" //max 15 characters without spaces
  #define SENSOR_PIN              13 // GPIO pin on which the float sensor is connected
  #define WATER_PRESENCE_SENSOR_PIN 12 // GPIO pin on which the water presence sensor is connected
  uint8_t gatewayAddress[] =      GATEWAY_SF_AP_MAC; //comes from secrets.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  // #define WiFi_SSID_PSWD       primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define DEFAULT_CHANNEL         1
  #define SLEEP_DURATION          30 // sleep time interval in seconds
  #define BATT_RESISTOR_CONST     5.3773 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R2
        // I have used R1 = 1.2M , R2=270K. calc factor comes to 5.444 but actual measurements gave me a more precise value of 5.3773
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
