
#ifndef FLOW_METER_CONFIG_H
#define FLOW_METER_CONFIG_H

#if defined(FLOW_METER_TANK)
  #warning "Compiling the program for the device: FLOW METER1"
  #define DEVICE_NAME             "water_flow_meter1"
  #define MQTT_BASE_TOPIC         "home/water/flow_meter_tank/"
  #define ESP_IP_ADDRESS          IPAddress(192,168,1,70)
  #define VOLUME_UNIT             LIT // could be one of LIT/KLIT (litres / Kilo Litres)

#else
  #error "Device type not selected, see flow_meter_config.h"
#endif

#endif
