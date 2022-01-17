
/*
 * Sketch for temperatire sensor using DS18B20 sensor. This device communicates over ESPNow and not in WiFi. 
 * The sketch waks up , supplies power to the senssor , reasa the temperature, creates a structure with all info and sends to the reciever (slave) ESP
 * It then goes to sleep for a defined period. In sleep mode a barebone ESP12 consumes ~20uA current and hence can run on batteries for a lond time.
 * Uses some header files from the includes folder (see include folder for those files)
 * TO DO :
 * - store the last used channel in the RTC memory , it currently spends ~2 sec in obtaining the channel
 * - have multiple slaves to which a message can be tranmitted in the order of preference
 */
//Specify the sensor this is being compiled for, see Config.h for list of all params for a sensor
//#define WEMOS_SENSOR
#define SOLAR_GEYSER_SENSOR

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "secrets.h"
#include "espnowMessage.h" // for struct of espnow message
#include "Config.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <averager.h>

#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#define WAIT_TIMEOUT 50 // time in millis to wait for acknowledgement of the message sent
#define MAX_MESSAGE_RETRIES 4 // No of times a message is retries to be sent before dropping the message
#define SLEEP_TIME 60e6 // sleep time interval in microseconds
#define ONE_WIRE_BUS 4 // gets readings from the data pin of DS18B20 sensor , there should be a pull up from this pin to Vcc
#define SENSOR_POWER_PIN 5 //supplies power to the sensor module , takes about 0.35mA , so can easily be sourced by GPIO
#define RESISTOR_CONST 5.61 // constant obtained by Resistor divider network. Vbat----R1---R2---GND . Const = (R1+R2)/R1
        // I have used R1 = 1M , R2=270K. calc factor comes to 4.7 but actual measurements gave me a more precise value of 5.6

#define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver

typedef struct esp_now_peer_info {// this is defined in esp_now.h for ESP32 but not in espnow.h for ESP8266
  u8 peer_addr[6];
  uint8_t channel;
  uint8_t encrypt;
}esp_now_peer_info_t;

espnow_message myData;
constexpr char WIFI_SSID[] = primary_ssid;// from secrets.h
volatile bool bSuccess = false;
volatile bool bResultReady = false;
char device_id[13];
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);            // Pass the oneWire reference to Dallas Temperature.

// MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
// You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address

/*
 * Gets the WiFi channel of the SSID of your router, It has to be on the same channel as this one as the receiver will listen to ESPNow messages on the same 
 * While theritically it should be possible for WiFi and ESPNow to work on different channels but it seems due to some bug (or behavior) this isnt possible with espressif
 */
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// callback when data is sent
esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) {
  bSuccess = (status == 0 ? true : false);
  DPRINT("\r\nLast Packet Send Status:\t");
  DPRINTLN(status == 0 ? "Delivery Success" : "Delivery Fail");
  bResultReady = true;
});

/*
 * Callback called on sending a message.
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void setup() {
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN();
  pinMode(SENSOR_POWER_PIN,OUTPUT);
  sensors.begin();//start sensors
  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);
// Sometimes after loading the new firmware the channel doesnt change to the required one, I have to reset the board to do the same.
// It rather reports the channel as 0 , I will have to look into this and solve, I think I saw some discussion around this where ESP remembers the last channel used 
// I think I have to fo a WiFi.disconnect to circumvent this, will try this out.

  WiFi.disconnect(); // trying to see if the issue of sometimes it not setting the right channel gets solved by this.
  // To change the channel you have to first call wifi_promiscuous_enable(true) , change channel and then call wifi_promiscuous_enable(false)
//  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  wifi_promiscuous_enable(true);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(false);
  delay(10);
  short ch = wifi_get_channel();
  DPRINT("channel:");DPRINTLN(ch);
  //strange behavior : If I define ch as byte and make the comparison below , the ESP resets due to WDT and then hangs
  if(ch == 0)
  {
    DPRINTLN("WiFi Channel not set properly, restarting");
    ESP.restart();
  }
//  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //Init ESP-NOW
  if (esp_now_init() != 0) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(MY_ROLE);
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  //Add peer , There is no method in ESP8266 to add a peer by passing esp_now_peer_info_t object unlike ESP32
  if (esp_now_add_peer((u8*)gatewayAddress, RECEIVER_ROLE, channel, NULL, 0) != 0){
    DPRINTLN("Failed to add peer");
    return;
  }

  String wifiMacString = WiFi.macAddress();
  wifiMacString.replace(":","");
  snprintf(device_id, 13, "%s", wifiMacString.c_str());
  strcpy(device_id,wifiMacString.c_str());
  DPRINTF("deviceid:%s\n",device_id);
  digitalWrite(SENSOR_POWER_PIN,HIGH);//set PIN high to given power to the sensor module
//  DPRINT("setup complete:");DPRINTLN(millis());
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
    for(short i = 0;i<MAX_MESSAGE_RETRIES;i++)
    {
      // supply power to the sensor 
      sensors.requestTemperatures(); // Send the command to get temperatures
      myData.floatvalue1 = sensors.getTempCByIndex(0); // get temperature reading for the first sensor
      DPRINT("Temp:");DPRINTLN(myData.floatvalue1);
      
      //measure battery voltage on ADC pin , average it over 10 readings
      averager<int> avg_batt_volt;
      for(short i=0;i<10;i++)
      {
        avg_batt_volt.append(analogRead(A0));
        delay(1);
      }
      
      float batt_level = mapf(avg_batt_volt.getAverage(), 0, 1023, 0, RESISTOR_CONST);
      //convert batt level into a %tage , 3.5 -> 0% , 4.2 -> 100% (min and max voltages of a Li-Ion battery
      if(batt_level <=3.5)
        myData.floatvalue2 = 0;
      else if(batt_level >= 4.2)
        myData.floatvalue2 = 100;
      else
        myData.floatvalue2 = mapf(batt_level,3.5,4.2,0,100);
        
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
      myData.intvalue1 = 0;
      myData.intvalue2 = 0;
      myData.intvalue3 = 0;
      myData.intvalue4 = 0;
      myData.floatvalue3 = 0;
      myData.floatvalue4 = 0;
      strcpy(myData.chardata1,"");
      strcpy(myData.chardata2,"");
      myData.message_id = millis();//there is no use of message_id so using it to send the uptime
        
      int result = esp_now_send(gatewayAddress, (uint8_t *) &myData, sizeof(myData));
      long waitTimeStart = millis();
      if (result == 0) {
        DPRINTLN("Sent with success");
      }
      else {
        DPRINTLN("Error sending the data");
      }
      while(!bResultReady && ((millis() - waitTimeStart) < WAIT_TIMEOUT))
      {
        delay(1);
      }
      bResultReady = false; // prepare for next iteration
      if(bSuccess)
      {
        break;
      }
    }
    digitalWrite(SENSOR_POWER_PIN,LOW);//remove power to the sensor module else it consumes power (~ 20uA more)
    DFLUSH();
    ESP.deepSleep(SLEEP_TIME);//ESP consumes ~20uA during deep sleep which is great!
}
