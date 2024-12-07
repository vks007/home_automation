
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
#include <averager.h>
#if USING(EEPROM_STORE)
#define EEPROM_SIZE 16 // number of bytes to be allocated to EEPROM
#include <EEPROM.h> // to store WiFi channel number to EEPROM
#endif

// ************ HASH DEFINES *******************
// ************ HASH DEFINES *******************

// ************ GLOBAL OBJECTS/VARIABLES *******************
#if USING(EEPROM_STORE)
  const char* ssid = WiFi_SSID; // comes from config.h
#endif
//const char* password = WiFi_SSID_PSWD; // comes from config.h
const uint64_t sleep_time = SLEEP_DURATION * 1e6; // sleep time in uS for the ESP in between readings
espnow_message myData;
char device_id[13];
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif
volatile bool waiting_for_msg_ack = false; // flag to indicate when we've sent the message and are waiting for its acknowledgment
short msg_ack_timeout = 10; // timeout in ms to wait for msg acknowlegment
ulong msg_sent_time = 0; // keeps the time as millis() on when the message was sent
volatile bool go_to_sleep = false;
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
  if (status == 0){
    DPRINTLN("Delivery success");
    //statusLED.blinkNumberOfTimes(500, 500, 1);
  }
  else{
    DPRINTFLN("Delivery fail:%d",status);
    //statusLED.blinkNumberOfTimes(500, 500, 3);
  }
  go_to_sleep = true;
  // BEAWARE , DONT USE ANY delay() statement on this function else the ESP will crash
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
  DPRINTFLN("This device's MAC add: %s",wifiMacString.c_str());

}

void setup() {
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN();
  printInitInfo();
  pinMode(SENSOR_PIN,INPUT); // the pin has an external pull up resistor for better noise immunity
  pinMode(WATER_PRESENCE_SENSOR_PIN,INPUT); // the pin has an external pull up resistor for better noise immunity

  DPRINTLN("initializing espnow");
//  initilizeESP(ssid,MY_ROLE);
  #if USING(EEPROM_STORE)
    //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
    EEPROM.begin(EEPROM_SIZE);// size of the EEPROM to be allocated, 16 is the minimum
    initilizeESP(ssid,MY_ROLE,DEFAULT_CHANNEL);
  #else
    initilizeESP(DEFAULT_CHANNEL,MY_ROLE,WIFI_STA);
  #endif

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
    myData.intvalue1 = !digitalRead(SENSOR_PIN);
    DPRINTFLN("sensor value: %d",myData.intvalue1);
    myData.intvalue2 = !digitalRead(WATER_PRESENCE_SENSOR_PIN);
    DPRINTFLN("water presence sensor value: %d",myData.intvalue2);

    //measure battery voltage on ADC pin , average it over some readings
    // NOTE: Calling analogRead() too frequently causes WiFi to stop working. When WiFi is under operation, 
    // analogRead() result may be cached for at least 5ms between effective calls. Ref: https://arduino-esp8266.readthedocs.io/en/latest/reference.html#analog-input
    averager<int> avg_batt_volt;
    for(short i=0;i<3;i++)
    {
      avg_batt_volt.append(analogRead(A0));
      delay(10);
    }

    short analog_value = avg_batt_volt.getAverage();
    float batt_level = (analog_value/1023.0)* BATT_RESISTOR_CONST;
    myData.floatvalue1 = batt_level;
    DPRINT("Battery:");DPRINTLN(myData.floatvalue1);
    myData.intvalue3 = analog_value; //just for debugging purposes

    //Set other values to send
    // If devicename is not given then generate one from MAC address stripping off the colon
    #ifndef DEVICE_NAME
      String wifiMacString = WiFi.macAddress();
      wifiMacString.replace(":","");
      snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
    #else
      strcpy(myData.device_name,DEVICE_NAME);
    #endif
    myData.intvalue4 = 0;
    myData.floatvalue2 = 0;
    myData.floatvalue3 = 0;
    myData.floatvalue4 = 0;
    strcpy(myData.chardata1,"");
    strcpy(myData.chardata2,"");
    myData.message_id = secureRandom(1000); // get a random value between 0 - 1000

    bool result = sendESPnowMessage(&myData,gatewayAddress,1,true);
    msg_sent_time = millis();
    DFLUSH();
    waiting_for_msg_ack = result == 0?true:false;
    return result;
}

void loop() {
  if(!waiting_for_msg_ack)
  {
    if(sendMessage() != 0)
    {
      // we failed to send the msg successfully, sleep and try again on wakeup
        go_to_sleep = true;
    }
  }
  if((millis() - msg_sent_time) > msg_ack_timeout)
  {
    //we sent the message but havent got the ack in alloted time, go to deep sleep
    go_to_sleep = true;
  }
  if(go_to_sleep == true)
  {
    DPRINTFLN("Entering deep sleep for %d secs",SLEEP_DURATION);
    DFLUSH();
    ESP.deepSleep(sleep_time);
  }

}
