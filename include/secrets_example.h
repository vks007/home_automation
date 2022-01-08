/*
 *secrets.h - file to store secret strings like WiFi credentials ets
 * Include this file into your programs and refer the variables here to pick up thier values
 * This file should not be checked into git. Advantage of this file is that you dont have to check in your secret info on git
*/

#ifndef SECRETS_H
#define SECRETS_H

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

#endif


