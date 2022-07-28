
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
#include "macros.h"
#include "secrets.h"
#include "Config.h"
#include "Debugutils.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h"
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller
#include <DallasTemperature.h>
#include <OneWire.h>
#include <averager.h>
#include <EEPROM.h>

// ************ HASH DEFINES *******************
#define MAX_COUNT 5000 // No of times a message is retries to be sent before dropping the message
#define ONE_WIRE_BUS 4 // gets readings from the data pin of DS18B20 sensor , there should be a pull up from this pin to Vcc
#define RESISTOR_CONST 5.156 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R1
        // I have used R1 = 1M , R2=270K. calc factor comes to 4.7 but actual measurements gave me a more precise value of 5.156
// ************ HASH DEFINES *******************

// ************ GLOBAL OBJECTS/VARIABLES *******************
espnow_message myData;
char device_id[13];
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);            // Pass the oneWire reference to Dallas Temperature.
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif
// ************ GLOBAL OBJECTS/VARIABLES *******************

// MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
// You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address

/*
Function to map float values in a range . map only operates on int values
*/
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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
  DPRINTLN("Starting up as a ESPNow Temperature sensor");
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


void setup() {
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN();
  printInitInfo();
  pinMode(SENSOR_POWER_PIN,OUTPUT);
  sensors.begin();//start sensors

  //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
  EEPROM.begin(16);// 16 is the size of the EEPROM to be allocated, 16 is the minimum

  DPRINTLN("initializing espnow");
  initilizeESP(WIFI_SSID,MY_ROLE);

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

  digitalWrite(SENSOR_POWER_PIN,HIGH);//set PIN high to given power to the sensor module
//  DPRINT("setup complete:");DPRINTLN(millis());

}


void loop() {
  for(short i = 0;i<MAX_COUNT;i++)
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
    // If the sensor is disconnected it gives junk negative values, to eliminate them I am setting the value to MIN_TEMP in that case
    if(sensors.getTempCByIndex(0) < MIN_TEMP)
      myData.floatvalue1 = MIN_TEMP;
    else
      myData.floatvalue1 = sensors.getTempCByIndex(0); // get temperature reading for the first sensor
    DPRINT("Temp:");DPRINTLN(myData.floatvalue1);
    
    //measure battery voltage on ADC pin , average it over 10 readings
    averager<int> avg_batt_volt;
    for(short i=0;i<10;i++)
    {
      avg_batt_volt.append(analogRead(A0));
      delay(5);
    }
    myData.intvalue1 = avg_batt_volt.getAverage(); // assigning this just for debug purposes

    float batt_level = (avg_batt_volt.getAverage()/1023.0)* RESISTOR_CONST;
    myData.floatvalue2 = batt_level;

    //Set other values to send
    // If devicename is not given then generate one from MAC address stripping off the colon
    if(DEVICE_NAME == "")
    {
      String wifiMacString = WiFi.macAddress();
      wifiMacString.replace(":","");
      snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
    }
    else
      strcpy(myData.device_name,DEVICE_NAME);
//    myData.intvalue1 = 0;
    myData.intvalue2 = 0;
    myData.intvalue3 = 0;
    myData.intvalue4 = 0;
    myData.floatvalue3 = 0;
    myData.floatvalue4 = 0;
    strcpy(myData.chardata1,"");
    strcpy(myData.chardata2,"");
    myData.message_id = millis();//there is no use of message_id so using it to send the uptime
      
    //int result = esp_now_send(gatewayAddress, (uint8_t *) &myData, sizeof(myData));
    int result = sendESPnowMessage(&myData,gatewayAddress,1,false);
    if (result == 0) {
      DPRINTLN("Delivered with success");}
    else {DPRINTFLN("Error sending/receipting the message, error code:%d",result);}
    
    #if(USING(TESTING)) // If we are testing then it sends a message MAX_COUNT times
    {
      delay(1);
    }
    #else 
        break;
    #endif
  }
  DPRINTLN("Going to sleep");
  DFLUSH();
  digitalWrite(SENSOR_POWER_PIN,LOW);//remove power to the sensor module else it consumes power (~ 20uA more)
  //ESP.deepSleep(SLEEP_DURATION*10e6); // This statement does not work - not sure why, I am going crazy behind this.
  // The solution is to hardcode the value for time instead of taking from a #define
  // The problem I face is that with the above statement , ESP never wakes up after the first sleep, sometimes it doesnt even start up
  ESP.deepSleep(30*10e6);

}
