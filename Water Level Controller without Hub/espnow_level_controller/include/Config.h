
#ifndef CONFIG_H
#define CONFIG_H

#include "macros.h"
// Define all your devices here and then pass the DEVICE in the build flags in platform.ini file
#define CONTROLLER_W8 1
#define CONTROLLER_TEST 2

#if (DEVICE == CONTROLLER_W8)
  //Turn features ON and OFF below start
  #define SERIAL_DEBUG            IN_USE // Debug statements in use or not , dont turn it ON if you are using RX pin for MOTION SENSOR
  #define SECURITY                NOT_IN_USE // encryption of messages
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not, at present only needed for sensor devices, gateway has not present use
  #define WIFI_CONNECTION         NOT_IN_USE // WiFi is being used or not
  //Turn features ON and OFF below ----------- end

  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "tank_controller_w8" //no spaces as this is used in topic names too
  #define ESP_IP_ADDRESS          IP_controllerW8 //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               w8_ssid //from secrets.h
  #define WiFi_SSID_PSWD          w8_ssid_pswd //from secrets.h
  #define DEFAULT_CHANNEL         1
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define SUMP_LED                12 // GPIO on which sump status LED is connected
  #define TANK_LED                13 // GPIO on which tank status LED is connected
  #define MOTOR_PIN               14 //GPIO on which the motor is connected
  #define SUMP_PIN                4 // GPIO on which the sump (underground water storage) sensor is connected
  #define DEVICE_MAC              CONTROLLER_W8_AP_MAC // GATEWAY_FF_AP_MAC //CONTROLLER_W8_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  #define INVERT_LEVEL_LOGIC       true
  #define SENSOR_HEALTH_INTERVAL  45e3 // interval is millisecs to wait for sensor values before rasing a disconnect event
#elif (DEVICE == CONTROLLER_TEST)
  //Turn features ON and OFF below start
  #define SERIAL_DEBUG            IN_USE // Debug statements in use or not , dont turn it ON if you are using RX pin for MOTION SENSOR
  #define SECURITY                NOT_IN_USE // encryption of messages
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not, at present only needed for sensor devices, gateway has not present use
  #define WIFI_CONNECTION         NOT_IN_USE // WiFi is being used or not
  //Turn features ON and OFF below ----------- end

  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "tank_controller_test" //no spaces as this is used in topic names too
  #define ESP_IP_ADDRESS          IP_controllerW8 //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               test_ssid //from secrets.h
  #define WiFi_SSID_PSWD          test_ssid_pswd //from secrets.h
  #define DEFAULT_CHANNEL         1
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define SUMP_LED                12 // GPIO on which sump status LED is connected
  #define TANK_LED                13 // GPIO on which tank status LED is connected
  #define MOTOR_PIN               14 //GPIO on which the motor is connected
  #define SUMP_PIN                4 // GPIO on which the sump (underground water storage) sensor is connected
  #define DEVICE_MAC              CONTROLLER_W8_AP_MAC // GATEWAY_FF_AP_MAC //CONTROLLER_W8_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  #define INVERT_LEVEL_LOGIC       false
  #define SENSOR_HEALTH_INTERVAL  30e3 // interval is millisecs to wait for sensor values before rasing a disconnect event
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
