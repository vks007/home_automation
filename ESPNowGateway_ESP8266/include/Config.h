
#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

// Define all your devices here and then pass the DEVICE in the build flags in platform.ini file
#define GATEWAY_GF 1
#define GATEWAY_FF 2
#define GATEWAY_TEST 3


#if (DEVICE == GATEWAY_GF)
  #define DEVICE_NAME             "gateway_gf" //no spaces as this is used in topic names too
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IP_gateway_gf // from secrets.h\static_ipaddress.h
  #define WiFi_SSID               gf_ssid //from secrets.h
  #define WiFi_SSID_PSWD          gf_ssid_pswd //from secrets.h
  #define STATUS_LED              2
  #define DEVICE_MAC              GATEWAY_GF_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
#elif (DEVICE == GATEWAY_FF)
  #define DEVICE_NAME             "gateway_ff" //no spaces as this is used in topic names too
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IP_gateway_ff //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define STATUS_LED              2 //GPIO on which the status led is connected
  #define DEVICE_MAC              GATEWAY_FF_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
#elif (DEVICE == GATEWAY_TEST)
  #define DEVICE_NAME             "gateway_test" //no spaces as this is used in topic names too
  #define MQTT_TOPIC              "home/espnow/" DEVICE_NAME
  #define MQTT_BASE_TOPIC          "home/espnow"
  #define ESP_IP_ADDRESS          IP_gateway_test //from secrets.h\static_ipaddress.h
  #define WiFi_SSID               primary_ssid //from secrets.h
  #define WiFi_SSID_PSWD          primary_ssid_pswd //from secrets.h
  #define STATUS_LED              2 //GPIO on which the status led is connected
  //#define API_TIMEOUT             60 // this defaults to 600 sec, you can override it here. This timeout is for monitoring of MQTT, beyond which the ESP restarts
  #define DEVICE_MAC              GATEWAY_TEST_MAC // from secrets.h . You should preferably define a custom MAC instead of actual device MAC so that the MAC doesnt change with device
#else
  #error "Device type not found. Have you passed DEVICE id in platform.ini as build flag. See Config.h for all DEVICES"
#endif

#endif
