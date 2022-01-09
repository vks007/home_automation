
#ifndef FLOW_METER_CONFIG_H
#define FLOW_METER_CONFIG_H

#if defined(TANK_FLOW_METER)
  #warning "Compiling the program for the device: TANK_FLOW_METER"
  #define DEVICE_ID              "tank_flow_meter"
  #define DEVICE_NAME             "Tank Flow Meter"
  #define MQTT_BASE_TOPIC         "home/water/" DEVICE_ID "/"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,78)
  #define GATEWAY1                default_gateway // comes from secrets.h
  #define SUBNET1                 subnet_mask // comes from secrets.h
  #define SSID                    primary_ssid // comes from secrets.h
  #define SSID_PSWD               primary_ssid_pswd // comes from secrets.h
  #define MQTT_SERVER1            mqtt_broker // comes from secrets.h
  #define MQTT_PORT1              mqtt_port // comes from secrets.h
  #define MQTT_USER1              mqtt_uname // comes from secrets.h
  #define MQTT_PSWD1              mqtt_pswd // comes from secrets.h

#else
  #error "Device type not selected, see flow_meter_config.h"
#endif

#endif
