#ifndef CONFIG_H
#define CONFIG_H

#if defined(WEMOS_SENSOR)
  #define DEVICE_NAME             "wemos_test" //max 15 characters without spaces
    uint8_t gatewayAddress[] = GATEWAY_GF_MAC; //comes from secrets.h
#elif defined(SOLAR_GEYSER_SENSOR)
  #define DEVICE_NAME             "solar_geyser" //no spaces as this is used in topic names too
    uint8_t gatewayAddress[] = GATEWAY_FF_MAC; //comes from secrets.h
#else
  #error "Device type not selected, see Config.h"
#endif

#endif
