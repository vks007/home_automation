#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

//common files from /libraries/MyFiles
#include "secrets.h" 
#include "version.h" 
//#include "myutils.h" //common utilities
#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#include "Debugutils.h" 

#define FLOW_METER_TANK
#include "flow_meter_config.h"

#define VERSION "1.0.0"
const char compile_version[] = VERSION __DATE__ " " __TIME__;

const char ssid[] = SSID1; //SSID of WiFi to connect to
const char ssid_pswd[] = SSID1_PSWD; //Password of WiFi to connect to

const char* deviceName = "flow_meter_tank";


#define STATUS_LED_PIN D1  
#define PULSE_PIN D7
#define REPORT_INTERVAL 2 // interval in seconds at which the message is posted to MQTT
#define DEBOUNCE_INTERVAL 10 //debouncing time in ms for interrupts
#define PULSE_PER_LIT 255 //no of pulses the meter counts for 1 lit of water
#define MAX_MQTT_CONNECT_RETRY 5
#define MAX_JSON_LEN  100 //max no of characters in a json doc
#define STATE_TOPIC "state"
#define WIFI_TOPIC "wifi"
#define DEBUG_TOPIC "debug"

volatile unsigned int Pulses = 0;
volatile unsigned int TotalPulses = 0;
volatile float flow_rate = 0.0;//flow rate is per minute
volatile float total_volume = 0.0;//stores the total volume of liquid since start
volatile float last_volume = 0.0;//stores the last value of total_volume 

#ifdef KLIT
  const float factor = 0.001;
#else  //LIT - default
  const float factor = 1.0;
#endif

os_timer_t publish_timer;
bool publish_tick = true;//to enable publishing of message on startup
volatile unsigned long LastMicros;
String ip_addr = "";

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void ICACHE_RAM_ATTR pulseHandler() {
  if((long)(micros() - LastMicros) >= DEBOUNCE_INTERVAL * 1000) {
    Pulses = Pulses + 1;
    LastMicros = micros();
  }
}

void timerCallback(void *pArg) {
  TotalPulses += Pulses;
  flow_rate = (float)Pulses/(PULSE_PER_LIT*REPORT_INTERVAL)*60.0*factor;
  last_volume = total_volume; //save last value
  total_volume = (float)TotalPulses/PULSE_PER_LIT*factor;
  Pulses = 0;//reset pulses as this is needed to calculate rate per REPORT_INTERVAL only
  if(last_volume != total_volume)
    publish_tick = true;
    //only publish a message if the values have changed, if flow rate changed, total volume would also change so no 
    //need to check both
}

void timerInit(void) {
  os_timer_setfn(&publish_timer, timerCallback, NULL);
  os_timer_arm(&publish_timer, 1000 * REPORT_INTERVAL, true);
}

void setupWiFi() 
{
  WiFi.config(ESP_IP_ADDRESS, GATEWAY1, SUBNET1);
  WiFi.hostname(DEVICE_NAME);// Set Hostname.
  //hostname += String(ESP.getChipId(), HEX);
  DPRINTLN("Hostname: " + WiFi.hostname());
  DPRINT("Connecting to ");DPRINT(ssid);
  
  WiFi.mode(WIFI_STA); // Force to station mode because if device was switched off while in access point mode it will start up next time in access point mode.
  WiFi.begin(ssid,ssid_pswd);
  for(short i=0;i<6000;i++) //break after 60 sec 6000*10 msec
  {
    if(WiFi.status() != WL_CONNECTED)
    {
      if(i%50 == 0)//print a progress dot every 500 ms
        DPRINT(".");
      delay(10);//Dont increase this delay, I set it to 500 and it takes a very long time to connect, I think this blocks the execution
    }
    else
    {
      DPRINTLN("");
      DPRINT("WiFi connected, IP Address:");
      DPRINTLN(WiFi.localIP());
      setupOTA();
      break;
    }
  }
  if(WiFi.status() != WL_CONNECTED){
    DPRINTLN("Could not connect to WiFi");}
}

void setupOTA()
{
  DPRINTLN("");
  DPRINT("Chip ID: 0x");
  DPRINTLN(String(ESP.getChipId(), HEX));
  
  ArduinoOTA.setHostname((const char *)WiFi.hostname().c_str());
  ArduinoOTA.onStart([]() { });// switch off all outputs while upgrading but dont store this interim state on EPROM
  ArduinoOTA.onEnd([]() { });// do a fancy thing with our board led at end
  ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });
  ArduinoOTA.begin();
  DPRINTLN("OTA Server setup successfully");
}


/*
 * Function to connect to MQTT is not connected, and then publish the message
 */
bool publishMessage(String topic, String msg,bool retain) {
  // Loop until we're reconnected
  char i = 0;
  while (!mqtt_client.connected()) 
  {
    i++;
    DPRINTLN("Attempting MQTT connection...");
    mqtt_client.connect(DEVICE_NAME,MQTT_USER1,MQTT_PSWD1);
    if(i >= MAX_MQTT_CONNECT_RETRY)
      break;
  }
  DPRINT(topic);DPRINT("-->");DPRINT(msg);
  //Now publish the message
  if(mqtt_client.connected())
  {
    if (mqtt_client.publish(topic.c_str(), msg.c_str(),retain))
    {
      DPRINTLN(" - OK");
      return true;
    }
    else 
    {
      DPRINTLN(" - FAIL");
      return false;
    }
  }
  else
    DPRINTLN("Failed to establish MQTT connection...");
  
}

bool publishJsonMessage(String topic,StaticJsonDocument<MAX_JSON_LEN> doc,bool retain) {
  String payload;
  serializeJson(doc,payload);
  return publishMessage(topic,payload,retain);

}

void publishWiFimsg()
{
  //prepare the json doc for the wifi message
  StaticJsonDocument<MAX_JSON_LEN> doc;
  ip_addr = WiFi.localIP().toString();
  doc["ip_address"] = ip_addr;
  doc["mac"] = WiFi.macAddress();
  doc["version"] = compile_version;
  publishJsonMessage(MQTT_BASE_TOPIC WIFI_TOPIC,doc,true);
}

void setup() {
  DBEGIN(115200);
  delay(1);
  DPRINTLN("");
  DPRINTLN(compile_version);

  setupWiFi();

  if((WiFi.status() == WL_CONNECTED))
  {
    mqtt_client.setServer(MQTT_SERVER1, MQTT_PORT1);
  }

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(PULSE_PIN, pulseHandler, RISING);

  timerInit();
  publishWiFimsg();
}

void loop() 
{
  if (publish_tick == true) 
  {
    StaticJsonDocument<MAX_JSON_LEN> doc;
    doc["flow_rate"] = flow_rate;
    doc["flow_unit"] = (factor == 1?"lit/min":"Klit/min");
    doc["total_volume"] = total_volume;
    doc["vol_unit"] = (factor == 1?"lit":"Klit");
    doc["uptime"] = (unsigned long)millis()/1000;

    publishJsonMessage(MQTT_BASE_TOPIC STATE_TOPIC,doc,false);

    #ifdef DEBUG
      //prepare the json doc for the wifi message
      doc.clear();
      doc["TotalPulses"] = TotalPulses;
      publishJsonMessage(MQTT_BASE_TOPIC DEBUG_TOPIC,doc,false);
    #endif    
    
    publish_tick = false;

    if(ip_addr != WiFi.localIP().toString())//means the IP add has changed
      publishWiFimsg();
  }

  digitalWrite(STATUS_LED_PIN, 0);

  yield();
}
