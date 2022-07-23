#ifndef CONFIG_H
#define CONFIG_H

#define GENERIC_SENSOR 1
#define SOLAR_GEYSER_SENSOR 2

#if (DEVICE == GENERIC_SENSOR)
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define TESTING                 IN_USE // defines if we're in testing mode in which the sensor keeps sending readings in a loop for MAX_COUNT
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "generic_sensor" //max 15 characters without spaces
  #define SENSOR_POWER_PIN        5 //supplies power to the temp sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  uint8_t gatewayAddress[] =      GATEWAY_TEST_AP_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] =    primary_ssid;// from secrets.h
  #define MIN_TEMP                0 // minimum temprature which you want to measure, this would be the lowest value it will send
  #define SLEEP_TIME              30 // sleep time interval in seconds
#elif (DEVICE == SOLAR_GEYSER_SENSOR)
  #define SERIAL_DEBUG            IN_USE
  #define SECURITY                NOT_IN_USE // using security or not to encrypt messages
  #define TESTING                 NOT_IN_USE // defines if we're in testing mode in which the sensor keeps sending readings in a loop for MAX_COUNT
  #define MY_ROLE                 ESP_NOW_ROLE_CONTROLLER              // set the role of this device: CONTROLLER, SLAVE, COMBO
  #define RECEIVER_ROLE           ESP_NOW_ROLE_SLAVE              // set the role of the receiver : CONTROLLER, SLAVE, COMBO
  #define DEVICE_NAME             "solar_geyser" //no spaces as this is used in topic names too
  #define SENSOR_POWER_PIN        5 //supplies power to the temp sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  uint8_t gatewayAddress[] =      GATEWAY_FF_AP_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] =    primary_ssid;// from secrets.h
  #define MIN_TEMP                0 // minimum temprature which you want to measure, this would be the lowest value it will send and also send it when it has invalid readings
  #define SLEEP_DURATION          180 // sleep time interval in seconds , see notes on its usage, this isnt working well
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
