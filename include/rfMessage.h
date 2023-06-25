
#pragma once

#include "myutils.h"

// Datatypes in Arduino : https://www.tutorialspoint.com/arduino/arduino_data_types.htm
// specifying the __attribute__((packed)) for a struct results in exactly the same bytes as the constituents without any padding
//typedef struct __attribute__((packed)) rf_msg{
//cannot be more than RH_ASK_MAX_MESSAGE_LEN long (60) 
struct rf_msg
{
  uint8_t  device_id; // Id of the transmitter
  char device_name[15]; // name of the device
  uint16_t msg_id; // unique message id
  bool     water_level; // water level , full - true , empty - false
  float    batt_level; // battery level as a float value
};

/*
* equal to operator for rf_msg struct
*/
bool operator==(const rf_msg& lhs, const rf_msg& rhs)
{
  return (xstrcmp(lhs.device_name,rhs.device_name) && lhs.msg_id==rhs.msg_id && \
  lhs.device_id==rhs.device_id && lhs.water_level==rhs.water_level && lhs.batt_level==rhs.batt_level);
}

/*
* not equal to operator for rf_msg struct
*/
bool operator!=(const rf_msg& lhs, const rf_msg& rhs)
{
  return !(lhs==rhs);
}

