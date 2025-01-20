#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

// you cant use strings so you have to use #defines for numeric values
// Ref :https://stackoverflow.com/questions/2335888/how-to-compare-strings-in-c-conditional-preprocessor-directives
//The following defines the device type for which the program is being compiled, this is passed as a compile time parameter in platform.ini
#define MAIN_DOOR 1
#define TERRACE_DOOR 2
#define BALCONY_DOOR 3
#define TEST_DOOR 4

#define LOGIC_NORMAL  1 // logic to hold power with GPIO HIGH and cut off power with GPIO LOW
#define LOGIC_INVERTED 2 // logic to hold power with GPIO LOW and cut off power with GPIO HIGH

#if (DEVICE == MAIN_DOOR) 
  #pragma message "Compiling the program for the device: MAIN_DOOR"
  #define TEST_MODE               NOT_IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE 
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     IN_USE // If OTA mode is in use or not
  #define STATUS_LED              IN_USE // If Status LED is used or not, affects battery
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "main_door"
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define HOLD_PIN                0  // defines hold pin (will hold power to the ESP).
  #define SIGNAL_PIN              3 //GPIO from which the input is read
  //State Mapping of SIGNAL_PIN0 SIGNAL_PIN1:: 11=>IDLE , 00=> SENSOR_WAKEUP , 01=> SENSOR OPEN , 10=> SENSOR CLOSED
  #define HOLDING_LOGIC           LOGIC_NORMAL
  #define LED_GPIO                2
  #define LED_INVERTED            false // If LED is Active HIGH , define as false , if Active LOW , define as true
  #define LED_ON_DURATION         0 // Duration in millisecs for which Status LED is ON. If 0 then it will be ON for the lenght of ESP wakeup. 
                                  // Irrespective of the value specified , it will not be ON for a time less than ESP wake time , ~ 90ms
                                  // Suggest to keep this at 0 to save battery or even dont use LED 
  uint8_t customMACAddress[] =    MAIN_DOOR_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h

#elif (DEVICE == TERRACE_DOOR)
  #pragma message "Compiling the program for the device: TERRACE_DOOR"
  #define TEST_MODE               NOT_IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE 
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     IN_USE // If OTA mode is in use or not
  #define STATUS_LED              NOT_IN_USE // If Status LED is used or not, affects battery
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE  // set the role of the receiver
  #define DEVICE_NAME             "terrace_door" // This becomes the postfix of the final MQTT topic under which messages are published
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define HOLD_PIN                0  // defines hold pin (will hold power to the ESP).
  #define SIGNAL_PIN              3 //GPIO from which the input is read
  //State Mapping of SIGNAL_PIN0 SIGNAL_PIN1:: 11=>IDLE , 00=> SENSOR_WAKEUP , 01=> SENSOR OPEN , 10=> SENSOR CLOSED
  // gateway MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
  // You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address
  // As a best practice you should define your own custom Soft MAC address so that you dont have to update all your sensors if you change the gateway device
  #define HOLDING_LOGIC           LOGIC_NORMAL
  uint8_t customMACAddress[] =    TERRACE_DOOR_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h

#elif (DEVICE == BALCONY_DOOR)
  #pragma message "Compiling the program for the device: BALCONY_DOOR"
  #define TEST_MODE               IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE 
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     IN_USE // If OTA mode is in use or not
  #define STATUS_LED              IN_USE // If Status LED is used or not, affects battery
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "balcony_door"
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define HOLD_PIN                5  // defines hold pin (will hold power to the ESP).
  #define SIGNAL_PIN              4 //GPIO from which the input is read
  #define BOUNCE_DELAY            1 // bounce delay in seconds, this is used for a bumby door which bounces a few times before settling on either open or closed
  #define HOLDING_LOGIC           LOGIC_INVERTED
  #define LED_GPIO                2
  #define LED_INVERTED            false // If LED is Active HIGH , define as false , if Active LOW , define as true
  #define LED_ON_DURATION         0 // Duration in millisecs for which Status LED is ON. If 0 then it will be ON for the lenght of ESP wakeup. 
                                  // Irrespective of the value specified , it will not be ON for a time less than ESP wake time , ~ 90ms
                                  // Suggest to keep this at 0 to save battery or even dont use LED 
  uint8_t customMACAddress[] =    BALCONY_DOOR_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h

#elif (DEVICE == TEST_DOOR)
  #pragma message "Compiling the program for the device: TEST_DOOR" 
  #define TEST_MODE               IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE 
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     NOT_IN_USE // If OTA mode is in use or not
  #define STATUS_LED              IN_USE // If Status LED is used or not, affects battery
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "test_door"
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define HOLD_PIN                5  // defines hold pin (will hold power to the ESP).
  #define SIGNAL_PIN              4 //GPIO from which the input is read
  #define BOUNCE_DELAY            1 // bounce delay in seconds, this is used for a bumby door which bounces a few times before settling on either open or closed
  #define HOLDING_LOGIC           LOGIC_NORMAL // LOGIC_NORMAL holds the pin HIGH to keep ESP ON , LOGIC_INVERTED hold the PIN low to keep ESP ON
  #define LED_GPIO                2
  #define LED_INVERTED            false // If LED is Active HIGH , define as false , if Active LOW , define as true
  #define LED_ON_DURATION         0 // Duration in millisecs for which Status LED is ON. If 0 then it will be ON for the lenght of ESP wakeup. 
                                  // Irrespective of the value specified , it will not be ON for a time less than ESP wake time , ~ 90ms
                                  // Suggest to keep this at 0 to save battery or even dont use LED 
  uint8_t customMACAddress[] =    TEST_DOOR_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h

#else
  #error "Device type not selected, see Config.h"
#endif

#endif
