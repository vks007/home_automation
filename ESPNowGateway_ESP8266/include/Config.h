
#ifndef CONFIG_H
#define CONFIG_H

#include "macros.h"
// Define all your devices here and then pass the DEVICE in the build flags in platform.ini file
#define GATEWAY_FF 1 // gateway on first floor
#define GATEWAY_TEST 2
#define GATEWAY_SF 3 // gateway on second floor

#if (DEVICE == GATEWAY_FF)
  //Turn features ON and OFF below start
  #define SERIAL_DEBUG            IN_USE // Debug statements in use or not , dont turn it ON if you are using RX pin for MOTION SENSOR
  #define SECURITY                NOT_IN_USE // encryption of messages
  #define MOTION_SENSOR           IN_USE // if a motion sensor is connected to the ESP as an optional sensor
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not, at present only needed for sensor devices, gateway has not present use
  #define ESPNOW_OTA_SERVER       IN_USE // If enabled this device acts as a trigger for ESPNOW OTA
  #define WEBSOCKETS              NOT_IN_USE // if using web sockets to see logs via the browser
  //Turn features ON and OFF below ----------- end

  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "gateway_ff" //no spaces as this is used in topic names too , cannot be more than 15 chars
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IP_gateway_ff //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define DEVICE_MAC              GATEWAY_FF_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  #define PIR_PIN                 3 // GPIO pin no where PIR sensor is connected
  #define MOTION_SENSOR_NAME      "family_room_motion"
  #define MOTION_ON_DURATION      15 // time in seconds for which motion value should remain ON after detecting motion
#elif (DEVICE == GATEWAY_TEST)
  //Turn features ON and OFF below
  #define SERIAL_DEBUG            IN_USE // Debug statements in use or not
  #define SECURITY                NOT_IN_USE // encryption of messages
  #define MOTION_SENSOR           NOT_IN_USE // if a motion sensor is connected to the ESP as a sensor
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not
  #define ESPNOW_OTA_SERVER       IN_USE // If using this ESP as a trigger for ESPNOW OTA
  //Turn features ON and OFF below ----------- end
  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "gateway_test" //no spaces as this is used in topic names too, cannot be more than 15 chars
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC         "home/espnow"
  #define ESP_IP_ADDRESS          IP_gateway_test //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define STATUS_LED              2 //GPIO on which the status led is connected
  //#define API_TIMEOUT           60 // this defaults to 600 sec, you can override it here. This timeout is for monitoring of MQTT, beyond which the ESP resets itself
  #define DEVICE_MAC              GATEWAY_TEST_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
#elif (DEVICE == GATEWAY_SF)
  //Turn features ON and OFF below start
  #define SERIAL_DEBUG            IN_USE // Debug statements in use or not , dont turn it ON if you are using RX pin for MOTION SENSOR
  #define SECURITY                NOT_IN_USE // encryption of messages
  #define MOTION_SENSOR           NOT_IN_USE // if a motion sensor is connected to the ESP as an optional sensor
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not, at present only needed for sensor devices, gateway has not present use
  #define ESPNOW_OTA_SERVER       NOT_IN_USE // If enabled this device acts as a trigger for ESPNOW OTA
  #define WEBSOCKETS              NOT_IN_USE // if using web sockets to see logs via the browser
  //Turn features ON and OFF below ----------- end

  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "gateway_sf" //no spaces as this is used in topic names too , cannot be more than 15 chars
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IP_gateway_sf //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define DEVICE_MAC              GATEWAY_SF_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  #define PIR_PIN                 3 // GPIO pin no where PIR sensor is connected
  #define MOTION_SENSOR_NAME      "headroom_motion"
  #define MOTION_ON_DURATION      15 // time in seconds for which motion value should remain ON after detecting motion
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
