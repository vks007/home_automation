
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
  //Turn features ON and OFF below ----------- end

  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "controller_w8" //no spaces as this is used in topic names too
  #define ESP_IP_ADDRESS          IP_controllerW8 //from secrets.h\static_ipaddress.h
//  #define WiFi_SSID               primary_ssid //from secrets.h
  #define DEFAULT_CHANNEL         1
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define MOTOR_PIN               16 //GPIO on which the motor is connected
//  #define DEVICE_MAC              CONTROLLER_W8_AP_MAC // GATEWAY_FF_AP_MAC //CONTROLLER_W8_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
#elif (DEVICE == CONTROLLER_TEST)
  //Turn features ON and OFF below
  #define SERIAL_DEBUG            IN_USE // Debug statements in use or not , dont turn it ON if you are using RX pin for MOTION SENSOR
  #define SECURITY                NOT_IN_USE // encryption of messages
  #define EEPROM_STORE            NOT_IN_USE // If EEPROM is in use or not, at present only needed for sensor devices, gateway has not present use
  #define WEBSOCKETS              NOT_IN_USE // if using web sockets to see logs via the browser
  //Turn features ON and OFF below ----------- end
  #define MY_ROLE                 ESP_NOW_ROLE_SLAVE              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_CONTROLLER              // set the role of the receiver
  #define DEVICE_NAME             "controller_test" //no spaces as this is used in topic names too
  #define ESP_IP_ADDRESS          IP_controllerTest //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define DEVICE_MAC              CONTROLLER_TEST_AP_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
