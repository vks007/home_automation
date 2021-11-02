
#ifndef DOOR_CONFIG_H
#define DOOR_CONFIG_H

#if defined(MAIN_DOOR)
  #warning "Compiling the program for the device: MAIN_DOOR"
  #define DEVICE_NAME             "main_door"
  #define MQTT_TOPIC              "home/main_door"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,50)
  #define WiFi_SSID               gf_ssid //from secrets.h
  #define WiFi_SSID_PSWD          gf_ssid_pswd //from secrets.h
  // define module type as either ESP_01 OR ESP_12 as the pin assignments are different for them
  #define ESP_01

#elif defined(TERRACE_DOOR)
  #warning "Compiling the program for the device: TERRACE_DOOR"
  #define DEVICE_NAME             "terrace_door"
  #define MQTT_TOPIC              "home/terrace_door"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,51)
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define ESP_01

#elif defined(BALCONY_DOOR)
  #warning "Compiling the program for the device: BALCONY_DOOR"
  #define DEVICE_NAME             "balcony_door"
  #define MQTT_TOPIC              "home/balcony_door"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,52)
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define ESP_12

#else
  #error "Device type not selected, see Door_config.h"
#endif

#endif
