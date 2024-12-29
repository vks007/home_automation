#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

// you cant use strings so you have to use #defines for numeric values
// Ref :https://stackoverflow.com/questions/2335888/how-to-compare-strings-in-c-conditional-preprocessor-directives
//The following defines the device type for which the program is being compiled, this is passed as a compile time parameter in platform.ini
#define TERRACE_FLOW_METER 1
#define TEST_FLOW_METER 2

#if (DEVICE == TERRACE_FLOW_METER)
  #pragma message "Compiling the program for the device: TERRACE_FLOW_METER" 
  #define TEST_MODE               IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     NOT_IN_USE // If OTA mode is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "terrace_flow_meter" // name of the device
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define PULSE_PIN                4  // defines the pin to which the encoder is connected
  uint8_t customMACAddress[] =    TERRACE_FLOW_METER_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h
#elif (DEVICE == TEST_FLOW_METER)
  #pragma message "Compiling the program for the device: TEST_FLOW_METER" 
  #define TEST_MODE               IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     NOT_IN_USE // If OTA mode is in use or not
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "test_flow_meter" // name of the device
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define PULSE_PIN                4  // defines the pin to which the encoder is connected
  #define WAKEUP_PIN               5  // defines the pin to which the wakeup signal is connected, in this case it is the same as the encoder pin 
  uint8_t customMACAddress[] =    TEST_FLOW_METER_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h
#else
  #error "Device type not selected, see Config.h"
#endif

#endif
