
#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

#if defined(GATEWAY_FF)
  #define DEVICE_NAME             "gateway_ff" //no spaces as this is used in topic names too
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,45)
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
#elif defined(GATEWAY_GF)
  #define DEVICE_NAME             "gateway_gf" //no spaces as this is used in topic names too
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,46)
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h

#else
  #error "Device type not selected, see Config.h"
#endif

#endif
