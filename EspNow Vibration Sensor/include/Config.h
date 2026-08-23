#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

// you cant use strings so you have to use #defines for numeric values
// Ref :https://stackoverflow.com/questions/2335888/how-to-compare-strings-in-c-conditional-preprocessor-directives
//The following defines the device type for which the program is being compiled, this is passed as a compile time parameter in platform.ini
#define GF_FLUSH 1

#if (DEVICE == GF_FLUSH) 
  #pragma message "Compiling the program for the device: GF_FLUSH" 
  #define TEST_MODE               IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            NOT_IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define OTA                     NOT_IN_USE // If OTA mode is in use or not
  #define STATUS_LED              IN_USE // If Status LED is used or not, affects battery
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "gf_flush"
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define HOLD_PIN                0  // defines hold pin (will hold power to the ESP).
  #define SIGNAL_PIN              12 // Rx, GPIO from which the input is read, this is tied to the output from the vibration sensor (via MOSFET)
  #define HOLDING_LOGIC           LOW // LOGIC LOW holds the pin LOW to keep ESP ON , LOGIC HIGH hold the PIN HIGH to keep ESP ON
  #define LED_GPIO                1 // not used as STATUS_LED is NOT_IN_USE
  #define LED_INVERTED            true // If LED is Active HIGH , define as false , if Active LOW , define as true
  #define LED_ON_DURATION         0 // Duration in millisecs for which Status LED is ON. If 0 then it will be ON for the lenght of ESP wakeup. 
                                  // Irrespective of the value specified , it will not be ON for a time less than ESP wake time , ~ 90ms
                                  // Suggest to keep this at 0 to save battery or even dont use LED 
  #define LED_BLINK_INTERVAL      1 // interval in seconds to blink the LED
  uint8_t customMACAddress[] =    GF_FLUSH_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h

#else
  #error "Device type not selected, see Config.h"
#endif

#endif
