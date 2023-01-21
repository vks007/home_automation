
/*
 * Sketch for temperatire sensor using DS18B20 sensor. This device communicates over ESPNow and not in WiFi. 
 * The sketch waks up , supplies power to the senssor , reasa the temperature, creates a structure with all info and sends to the reciever (slave) ESP
 * It then goes to sleep for a defined period. In sleep mode a barebone ESP12 consumes ~20uA current and hence can run on batteries for a lond time.
 * Uses some header files from the includes folder (see include folder for those files)
 * Features:
 * - Uses Deep Sleep to conserver power for most of the time
 * 
 * 
 * TO DO :
 * - have multiple slaves to which a message can be tranmitted in the order of preference
 */
//Specify the sensor this is being compiled for in platform.ini, see Config.h for list of all devices this can be compiled for

#include <Arduino.h>
#include "secrets.h"
#include "Config.h"
#include "Debugutils.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h"
#if USING(EEPROM_STORE)
#define EEPROM_SIZE 16 // number of bytes to be allocated to EEPROM
#include <EEPROM.h> // to store WiFi channel number to EEPROM
#endif

// ************ HASH DEFINES *******************
// ************ HASH DEFINES *******************

// ************ GLOBAL OBJECTS/VARIABLES *******************
const char* ssid = WiFi_SSID; // comes from config.h
const char* password = WiFi_SSID_PSWD; // comes from config.h
espnow_message myData;
char device_id[13];
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif
unsigned long lastDebounceTime = 0;  // the last time the output pin toggled
unsigned long debounceDelay = 500;    // the debounce time;
volatile bool button_pressed = false;
bool button_state = false;
// ************ GLOBAL OBJECTS/VARIABLES *******************
// need to include this file after ssid variable as I am using ssid inside espcontroller, not a good design but will sort this out later
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller

// MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
// You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address

/*
 * Callback when data is sent , It sets the bResultReady flag to true on successful delivery of message
 * The flag is set to false in the main loop where data is sent and then the code waits to see if it gets set to true, if not it retires to send
 */
esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) {
  deliverySuccess = status;
  DPRINT("OnDataSent:Last Packet delivery status:\t");
  DPRINTLN(status == 0 ? "Success" : "Fail");
  bResultReady = true;
});

/*
 * Callback called when a message is received , nothing to do here for now , just log message
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void printInitInfo()
{
  DPRINTLN("Starting up as a ESPNow Tank sensor");
  #if USING(SECURITY)
    DPRINTLN("Security ON");
  #else
    DPRINTLN("Security OFF");
  #endif
  String wifiMacString = WiFi.macAddress();
  // wifiMacString.replace(":","");
  // snprintf(device_id, 13, "%s", wifiMacString.c_str());
  // strcpy(device_id,wifiMacString.c_str());
  // DPRINTF("deviceid:%s\n",device_id);
  DPRINTFLN("This device's MAC add: %s",wifiMacString.c_str());

}

void ICACHE_RAM_ATTR sensor_changed() {
  if(!button_pressed)
    button_pressed = true;
}

void setup() {
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN();
  printInitInfo();
  pinMode(SENSOR_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_changed, FALLING);

  //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
  EEPROM.begin(EEPROM_SIZE);// size of the EEPROM to be allocated, 16 is the minimum

  DPRINTLN("initializing espnow");
  initilizeESP(ssid,MY_ROLE);

  #if(USING(SECURITY))
    esp_now_set_kok(kok, 16);
  #endif

  // register callbacks for events when data is sent and data is received
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  #if(USING(SECURITY))
    refreshPeer(gatewayAddress, key,RECEIVER_ROLE);
  #else
    refreshPeer(gatewayAddress, NULL,RECEIVER_ROLE);
  #endif

}

bool sendMessage()
{
    //button_state = !button_state;
    delay(10);
    button_state = digitalRead(SENSOR_PIN);
    myData.intvalue1 = button_state;
    DPRINTFLN("sensor value: %d",myData.intvalue1);

    //Set other values to send
    // If devicename is not given then generate one from MAC address stripping off the colon
    #ifndef DEVICE_NAME
      String wifiMacString = WiFi.macAddress();
      wifiMacString.replace(":","");
      snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
    #else
      strcpy(myData.device_name,DEVICE_NAME);
    #endif
    myData.intvalue2 = 0;
    myData.intvalue3 = 0;
    myData.intvalue4 = 0;
    myData.floatvalue1 = 0;
    myData.floatvalue2 = 0;
    myData.floatvalue3 = 0;
    myData.floatvalue4 = 0;
    strcpy(myData.chardata1,"");
    strcpy(myData.chardata2,"");
    myData.message_id = millis();//there is no use of message_id so using it to send the uptime

    bool result = sendESPnowMessage(&myData,gatewayAddress,1,true);
    if (result == 0) 
      {DPRINTLN("Delivered with success");}
    else 
      {DPRINTLN("Message not delivered");}
    
    return result;
}

void loop() {
    
  if(button_pressed)
  {
    DPRINTLN("button pressed");
    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
      lastDebounceTime = millis();
      sendMessage();
    }
    // else
    //   DPRINTLN("debouncing ignore...");
    button_pressed = false;
  }
  

}
