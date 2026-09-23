#ifndef CONFIG_H
#define CONFIG_H
#include "macros.h"

// you cant use strings so you have to use #defines for numeric values
// Ref :https://stackoverflow.com/questions/2335888/how-to-compare-strings-in-c-conditional-preprocessor-directives
//The following defines the device type for which the program is being compiled, this is passed as a compile time parameter in platform.ini
#define FF_FLUSH 1

#if (DEVICE == FF_FLUSH) 
  #pragma message "Compiling the program for the device: FF_FLUSH" 
  #define TEST_MODE               NOT_IN_USE // defines if this code is being used during testing, in this case, the ESP does not power down
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define EEPROM_STORE            IN_USE // If EEPROM is in use or not , it is needed if using OTA
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver
  #define DEVICE_NAME             "ff_flush"
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd // used only for OTA updates else this is not used , from secrets.h
  #define ESP_IP_ADDRESS          IP_espnow_sensor //from secrets.h\static_ipaddress.h
  #define SENSOR_POWER_PIN        4  // Pin which provides power to the rest of the sensor circuit
  #define SENSOR_POWER_LOGIC      HIGH // LOGIC LOW turns off the sensor power, LOGIC HIGH turns on the sensor power
  #define DEFAULT_CHANNEL         11
  #define SLEEP_DURATION          30 // deep sleep duration in seconds between idle checks/heartbeats when no vibration is present
  uint8_t customMACAddress[] =    FF_FLUSH_AP_MAC; // from secrets.h . Prefer defining a custom MAC instead of actual device MAC so that the MAC doesnt change with device
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h

  // ---- Vibration sensing config ----
  // Note: ADC_MODE(ADC_VCC) is NOT used in this sketch because the ESP8266 has a single ADC pin
  // and it cannot be shared between VCC measurement and an external analog signal (A0). The piezo/op-amp
  // signal is read via analogRead(A0), so no battery voltage is reported by this device.
  #define ADC_NOISE_THRESHOLD      30   // raw ADC reading (0-1023) above which a sample is considered "vibration present"
  #define ADC_SAMPLE_COUNT         2   // no of ADC samples taken in a burst to decide if vibration is present
  #define ADC_SAMPLE_INTERVAL_MS   10    // delay in ms between samples within a burst
  #define VIBRATION_POLL_INTERVAL_MS   5000  // how often (ms) to re-check the sensor while a vibration event is ongoing
  #define TEST_MESSAGE_INTERVAL_MS 1000 // how often (ms) to send sensor readings when TEST_MODE is enabled

#else
  #error "Device type not selected, see Config.h"
#endif

#endif
