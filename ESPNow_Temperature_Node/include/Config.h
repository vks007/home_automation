#ifndef CONFIG_H
#define CONFIG_H

#define GENERIC_SENSOR 1
#define SOLAR_GEYSER_SENSOR 2
#define NODEMCU_SENSOR 3

#if (DEVICE == GENERIC_SENSOR)
  #define DEVICE_NAME             "generic_sensor" //max 15 characters without spaces
  #define SENSOR_POWER_PIN        5 //supplies power to the sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] = primary_ssid;// from secrets.h
#elif (DEVICE == SOLAR_GEYSER_SENSOR)
  #define DEVICE_NAME             "solar_geyser" //no spaces as this is used in topic names too
  #define SENSOR_POWER_PIN        5 //supplies power to the sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] = primary_ssid;// from secrets.h
#elif (DEVICE == NODEMCU_SENSOR)
  #define DEVICE_NAME             "nodemcu_sensor" //max 15 characters without spaces
  #define SENSOR_POWER_PIN        5 //supplies power to the sensor module , takes about 0.35mA , so can easily be sourced by GPIO
  uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h
  constexpr char WIFI_SSID[] = gf_ssid;// from secrets.h
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
