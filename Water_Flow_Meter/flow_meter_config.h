
#ifndef FLOW_METER_CONFIG_H
#define FLOW_METER_CONFIG_H

#if defined(TANK_FLOW_METER)
  #warning "Compiling the program for the device: TANK_FLOW_METER"
  #define DEVICE_ID              "tank_flow_meter"
  #define DEVICE_NAME             "Tank Flow Meter"
  #define MQTT_BASE_TOPIC         "home/water/"DEVICE_ID"/"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,78)
  #define VOLUME_UNIT             LIT // could be one of LIT/KLIT (litres / Kilo Litres)

#else
  #error "Device type not selected, see flow_meter_config.h"
#endif

#endif
