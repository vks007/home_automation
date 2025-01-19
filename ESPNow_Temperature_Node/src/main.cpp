
/*
 * Sketch for temperature sensor using DS18B20 sensor. This device communicates over ESPNow and not in WiFi. 
 * The sketch wakes up, supplies power to the sensor , reads the temperature, creates a structure with all info and sends to the espnow reciever gateway
 * The gateway is connected to the WiFi and sends the data to the MQTT broker which is in turn integrated with Home Assistant
 * It then goes to sleep for a defined period. In sleep mode a barebone ESP12 consumes ~20uA current and hence can run on batteries for a long time.
 * Uses some header files from the includes folder (see include folder for those files)
 * Data Format: data is transmitted in the form of a structure espnow_message which has the following fields
 * espnow_message.sender_mac : mac of this device
 * espnow_message.message_id : uptime of the device in milliseconds
 * espnow_message.device_name : name of the device
 * espnow_message.intvalue1 : sleep duration
 * espnow_message.intvalue2 : not used
 * espnow_message.intvalue3 : not used
 * espnow_message.intvalue4 : not used
 * espnow_message.floatvalue1 : temperature reading
 * espnow_message.floatvalue2 : battery voltage reading
 * espnow_message.floatvalue3 : not used
 * espnow_message.floatvalue4 : not used
 * espnow_message.chardata1 : not used
 * espnow_message.chardata2 : not used
 * 
 * TO DO :
 * - have multiple slaves to which a message can be tranmitted in the order of preference
 * - Implement storing of message_id in the RTC memory so that it is not lost on restart
 */
//Specify the sensor this is being compiled for in platform.ini, see Config.h for list of all devices this can be compiled for

#include <Arduino.h>
#include "secrets.h"
#include "Config.h"
#include "Debugutils.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for espnow capabilities, has wrapper functions to send and receive messages
#include "myutils.h" // for utility functions
#include <DallasTemperature.h>
#include <OneWire.h>
#include <averager.h> // for averaging the battery voltage readings
#if USING(EEPROM_STORE)
#define EEPROM_SIZE 16 // number of bytes to be allocated to EEPROM
#include <EEPROM.h> // to store WiFi channel number to EEPROM
#endif

// ************ HASH DEFINES *******************
#define MAX_COUNT 5000 // No of times to loop in testing mode and send the message
#define VERSION "1.1"
// ************ HASH DEFINES *******************

// ************ GLOBAL OBJECTS/VARIABLES *******************
const char* ssid = WiFi_SSID; // comes from config.h
const char* password = WiFi_SSID_PSWD; // comes from config.h
//const uint64_t sleep_time = SLEEP_DURATION * 1e6; // sleep time in uS for the ESP in between readings
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the strings adjacent to each other become pasted together as one long string
espnow_message myData;
char device_id[13];
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensors(&oneWire);            // Pass the oneWire reference to Dallas Temperature.
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif
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
  DPRINTFLN("Version:%s",compile_version);
  DPRINTFLN("Starting up as device:%s",DEVICE_NAME);
  #if USING(SECURITY)
    DPRINTLN("Security status ON");
  #else
    DPRINTLN("Security status OFF");
  #endif
  String wifiMacString = WiFi.macAddress();
  DPRINTFLN("This device's MAC add: %s",wifiMacString.c_str());
}

//ADC_MODE(ADC_VCC);             /* measure Vcc */
// See solution to measure Vcc as well as sensor value on A0 here : https://arduino.stackexchange.com/questions/52952/read-both-battery-voltage-and-analog-sensor-value-with-nodemcu-esp8266

void setup() {
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN();
  printInitInfo();
  pinMode(SENSOR_POWER_PIN,OUTPUT);
  digitalWrite(SENSOR_POWER_PIN,HIGH);//set PIN high to given power to the sensor module
  sensors.begin();//start temperature sensor

  DPRINTLN("initializing espnow");
  #if USING(EEPROM_STORE)
    //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
    EEPROM.begin(EEPROM_SIZE);// size of the EEPROM to be allocated, 16 is the minimum
    initilizeESP(ssid,MY_ROLE,DEFAULT_CHANNEL);
  #else // this will not reply on a SSID and channel stored in EEPROM
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

float getBatteryVoltage()
{
  averager<int> avg_batt_volt;
  int temp = 0;
  for(short i=0;i<5;i++)
  {
    temp = analogRead(PIN_A0);
    //DPRINT("A0:");DPRINTLN(temp);
    avg_batt_volt.append(temp);
    delay(5);
  }
  return (avg_batt_volt.getAverage()/1023.0)* RESISTOR_CONST;
}

float getTemperature()
{
  float temp = 0;
  sensors.requestTemperatures(); // Send the command to get temperatures
  // If the sensor is disconnected it gives junk negative values, to eliminate them I am setting the value to MIN_TEMP in that case
  if(sensors.getTempCByIndex(0) < MIN_TEMP)
    temp = MIN_TEMP;
  else
    temp = sensors.getTempCByIndex(0); // get temperature reading for the first sensor
  DPRINT("Temp:");DPRINTLN(myData.floatvalue1);
  return temp;
}

void loop() {
  for(short i = 0;i<MAX_COUNT;i++) // this loop is for testing mode only, in normal case it will send the message only once
  {
    myData.floatvalue1 = getTemperature();
    myData.floatvalue2 = getBatteryVoltage();
    DPRINT("Battery:");DPRINTLN(myData.floatvalue2);

    //Set other values to send
    strcpy(myData.sender_mac,WiFi.macAddress().c_str()); //WiFi.softAPmacAddress()
    // If devicename is not given then generate one from MAC address stripping off the colon
    #ifndef DEVICE_NAME
      String wifiMacString = WiFi.macAddress();
      wifiMacString.replace(":","");
      snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
    #else
      strcpy(myData.device_name,DEVICE_NAME);
    #endif
    myData.intvalue1 = SLEEP_DURATION; // for debugging purposes
    myData.intvalue2 = 0;
    myData.intvalue3 = 0;
    myData.intvalue4 = 0;
    myData.floatvalue3 = 0;
    myData.floatvalue4 = 0;
    myData.chardata1[MAX_CHAR_DATA_LEN-1] = '\0'; // Ensure null termination (index starts from zero)
    short str_len = strlen(compile_version);
    if (str_len > MAX_CHAR_DATA_LEN-1)
        str_len = MAX_CHAR_DATA_LEN-1;
    strncpy(myData.chardata2, compile_version, str_len);
    myData.chardata2[str_len] = '\0'; // Ensure null termination
    myData.message_id = millis();//there is no use of message_id so using it to send the uptime

    bool result = sendESPnowMessage(&myData,gatewayAddress,1,true);
    if (result == 0) 
      {DPRINTLN("Delivered with success");}
    else 
      {DPRINTLN("Message not delivered");}
    
    #if(USING(TESTING_MODE)) // If we are testing then it sends a message MAX_COUNT times
    {
      delay(15000);
    }
    #else 
        break;
    #endif
  }
  DPRINTFLN("Going to sleep for %d secs",SLEEP_DURATION);
  DFLUSH();
  digitalWrite(SENSOR_POWER_PIN,LOW);//remove power to the sensor module else it consumes power (~ 20uA more)
  ESP.deepSleep(sleep_time);
  //ESP.deepSleep(SLEEP_DURATION*10e6); // This statement does not work - not sure why, It seems deepsleep expects a integer and cannot accept calculations
  // The strange thing that happens if I use the above statement is that the ESP goes into a trance mode and never comes up. 
  // Even worse as I have a large cap 1000uF across the ESP power, even if I disconnect and reconnect the power the ESP does not come back unless
  // I drain the cap and then supply power again

}
