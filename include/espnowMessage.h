
#ifndef ESPNOW_MESSAGE_H
#define ESPNOW_MESSAGE_H

// Datatypes in Arduino : https://www.tutorialspoint.com/arduino/arduino_data_types.htm
// ESPNow has a limit of sending 250 bytes MAX and so the struct should not be more than that
// specifying the __attribute__((packed)) for a struct results in exactly the same bytes as the constituents without any padding
//typedef struct __attribute__((packed)) espnow_message{
typedef struct espnow_message{
  char device_name[16];// contains unique device name, falls back to the mac address without colon if device id is not provided
  unsigned long message_id; //unique message id generated for each message
  int intvalue1; // int any data
  int intvalue2; // int any data
  int intvalue3; // // int any data
  int intvalue4; // // int any data
  float floatvalue1;// float data
  float floatvalue2;// float data
  float floatvalue3;// float data
  float floatvalue4;// float data
  char chardata1[16];// any char data
  char chardata2[16];// any char data
}espnow_message;

#endif