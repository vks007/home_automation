/*
 *secrets.h - file to store secret strings like WiFi credentials ets
 * Include this file into your programs and refer the variables here to pick up thier values
 * This file should not be checked into git. Advantage of this file is that you dont have to check in your secret info on git
*/

#ifndef SECRETS_H
#define SECRETS_H
#include "static_ipaddress.h"

#define primary_ssid "YOUR SSID"
#define primary_ssid_pswd "YOUR PSWD"
#define gf_ssid "YOUR SSID2"
#define gf_ssid_pswd "YOUR PSWD2"
#define default_gateway IPAddress(192,168,1,1)
#define default_dns IPAddress(192,168,1,1)
#define subnet_mask IPAddress(255,255,255,0)
#define mqtt_broker IPAddress(192,168,1,XXX)
#define mqtt_port 1883
#define mqtt_uname "XXX"
#define mqtt_pswd "XXXX"
#define ota_pswd "XXXX"
#define api_pswd "XXXX"
#define captive_portal_pswd "XXXX"
#define primary_mesh_name "XXXX"
#define primary_mesh_pswd "XXXX"
#define primary_mesh_port 4326
#define PMK_KEY_STR {0x11, 0x22, 0x322, 0x43, 0x54, 0x66, 0x77, 0x88, 0xF9, 0x11, 0xA1, 0x22, 0x32, 0x44, 0x45, 0x76}
#define LMK_KEY_STR {0x33, 0x43, 0x33, 0x14, 0x22, 0x14, 0x33, 0xF4, 0x13, 0x44, 0xA3, 0x14, 0x73, 0x44, 0x3C, 0x57}
#define CONTROLLERS { \
	{0x4C, 0x64, 0x33, 0x22, 0x44, 0x2D} ,\
	{0x3C, 0x2D, 0x12, 0x73, 0x41, 0x88}\
	}
//Define custom MAC addresses of your espnow devices, this has an advantage that you can change your device without affecting any code/configuration
// First 3 octet of the MAC are Manufacturer's ID , second 3 octet are device ID
// MAC addresses of gateway devices
#define GATEWAY_FF_AP_MAC {0x62, 0x01, 0x94, 0xFF, 0xFF, 0x01}//62:01:94:AB:16:01 - This is a custom SoftAP MAC addr of the ESP01 FF Gateway (defined randomly) - This is currently in use
#define GATEWAY_TEST_AP_MAC  {0x62, 0x01, 0x94, 0xFF, 0xFF, 0x02}//62:01:94:AB:16:02 - This is custom mac defined for testing
#define BROADCAST_AP_MAC  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}// broadcast MAC - to everyone
#define ESP32_DEVKIT_AP_MAC  {0x24, 0x0A, 0xC4, 0xF9, 0xCB, 0xD4}//62:01:94:AB:16:03

// MAC addresses of sensor devices
#define MAIN_DOOR_AP_MAC {0x62, 0x01, 0x94, 0xFF, 0x01, 0x01}//62:01:94:AB:15:01 - This is a custom SoftAP MAC addr
#define TERRACE_DOOR_AP_MAC {0x62, 0x01, 0x94, 0xFF, 0x01, 0x02}//62:01:94:AB:15:01
#define BALCONY_DOOR_AP_MAC {0x62, 0x01, 0x94, 0xFF, 0x01, 0x03}//62:01:94:AB:15:03
#define TEST_DOOR_AP_MAC {0x62, 0x01, 0x94, 0xFF, 0x01, 0x04}//62:01:94:AB:15:04
#define SOLAR_SENSOR_AP_MAC {0x62, 0x01, 0x94, 0xFF, 0x01, 0x05}//62:01:94:AB:15:05

#endif


