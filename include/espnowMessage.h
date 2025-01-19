
#ifndef ESPNOW_MESSAGE_H
#define ESPNOW_MESSAGE_H
#include "myutils.h"

#define OTA_MSG "OTA" // ota message , if received triggers an OTA mode
#define MAX_DEVICE_NAME_LEN 15 // max length of device name
#define MAX_CHAR_DATA_LEN 64 // max length of character data in the espnow_message struct

typedef enum {
    ESPNOW_SENSOR     = 0, // messages sent by sensor devices with their data
    ESPNOW_OTA        = 1, // messages sent by any device on OTA state
    ESPNOW_COMMAND    = 2,  // messages sent by any device with action info in the message
    ESPNOW_QUERY      = 3,  // messages to query the gateway about its info like channel number etc
    ESPNOW_RESPONSE      = 4  // messages in response to a query message
} msg_type_t;

typedef enum {
    MODE_NORMAL           = 0,
    MODE_OTA_START        = 125, // to avoid junk values read back from EEPROM matching this value
    MODE_OTA_END          = 126
} espnow_mode_t;

typedef struct espnow_device
{
  uint8_t mac[6];
  bool ota_mode;
  volatile bool ota_done;
  short duration;
  short interval;
}espnow_device;

// Datatypes in Arduino : https://www.tutorialspoint.com/arduino/arduino_data_types.htm
// ESPNow has a limit of sending 250 bytes MAX and so the struct should not be more than that
// specifying the __attribute__((packed)) for a struct results in exactly the same bytes as the constituents without any padding
//typedef struct __attribute__((packed)) espnow_message{
typedef struct espnow_message{
  char sender_mac[18]=""; // contains the mac addresss of the sender in the format "XX:XX:XX:XX:XX:XX"
  char device_name[MAX_DEVICE_NAME_LEN+1]="";// contains unique device name, falls back to the mac address without colon if device id is not provided
  unsigned long message_id; //unique message id generated for each message
  msg_type_t msg_type;
  int intvalue1; // int any data
  int intvalue2; // int any data
  int intvalue3; // // int any data
  int intvalue4; // // int any data
  float floatvalue1;// float data
  float floatvalue2;// float data
  float floatvalue3;// float data
  float floatvalue4;// float data
  char chardata1[64]="";// any char data
  char chardata2[64]="";// any char data
}espnow_message;

/*
* equal to operator for espnow_message struct
*/
bool operator==(const espnow_message& lhs, const espnow_message& rhs)
{
  return (xstrcmp(lhs.device_name,rhs.device_name) && lhs.message_id==rhs.message_id && \
  lhs.intvalue1==rhs.intvalue2 && lhs.intvalue1==rhs.intvalue2 && lhs.intvalue3==rhs.intvalue3 && \
  lhs.intvalue4==rhs.intvalue4 && lhs.floatvalue1==rhs.floatvalue1 && lhs.floatvalue2==rhs.floatvalue2 && \
  lhs.floatvalue3==rhs.floatvalue3 && lhs.floatvalue4==rhs.floatvalue4 && xstrcmp(lhs.chardata1,rhs.chardata1) && \
  xstrcmp(lhs.chardata2,rhs.chardata2));
}

/*
* not equal to operator for espnow_message struct
*/
bool operator!=(const espnow_message& lhs, const espnow_message& rhs)
{
  return !(lhs==rhs);
}

#endif