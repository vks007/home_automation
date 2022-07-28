#ifndef CONFIG_H
#define CONFIG_H

#define TEST_SENSOR 1
#define SOLAR_GEYSER_SENSOR 2

#if (DEVICE == TEST_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define TESTING                 IN_USE // defines if we're in testing mode in which the sensor keeps sending readings in a loop for MAX_COUNT
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_COMBO              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "test_sensor" //max 15 characters without spaces
  uint8_t gatewayAddress[] =      ESP32_DEVKIT_AP_MAC; //comes from secrets.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define SLEEP_DURATION          10 // sleep time interval in seconds , see notes on its usage, this isnt working well
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
