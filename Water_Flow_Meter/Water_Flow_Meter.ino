#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <ArduinoJson.h>

//common files from /libraries/MyFiles
#include "secrets.h" 
#include "version.h" 
//#include "myutils.h" //common utilities
#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#include "Debugutils.h" 


//#define DPRINT(x) do { Serial.print(x); } while (0)
//#define DPRINTLN(x) do { Serial.println(x); } while (0)

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

volatile unsigned int Pulses = 0;
volatile unsigned int PulsesLast = 0;
volatile unsigned int TotalPulses = 0;
volatile unsigned int PulsesPeriods = 0;
volatile float flow_rate = 0.0;//flow rate is per minute
volatile float total_volume = 0.0;//stores the total volume of liquid since start

#ifdef KLIT
  const float factor = 0.001;
#else  //LIT - default
  const float factor = 1.0;
#endif

os_timer_t publish_timer;
bool publish_tick = false;
volatile unsigned long LastMicros;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void ICACHE_RAM_ATTR pulseHandler() {
  if((long)(micros() - LastMicros) >= DEBOUNCE_INTERVAL * 1000) {
    Pulses = Pulses + 1;
    LastMicros = micros();
  }
}


void timerCallback(void *pArg) {
  PulsesLast = Pulses;
  TotalPulses += Pulses;
  flow_rate = (float)Pulses/(PULSE_PER_LIT*REPORT_INTERVAL)*60.0*factor;
  total_volume = (float)TotalPulses/PULSE_PER_LIT*factor;
/*  
  DPRINTFLN("Pulses:%d",Pulses);
  DPRINT("TotalPulses:");DPRINTLN(TotalPulses);
  DPRINTFLN("flow_rate:%f",flow_rate);
  DPRINTFLN("total_volume:%f",total_volume);
*/
  Pulses = 0;
  PulsesPeriods++;
  publish_tick = true;
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
    DPRINT("Attempting MQTT connection...");
    mqtt_client.connect(DEVICE_NAME,MQTT_USER1,MQTT_PSWD1);
    if(i >= MAX_MQTT_CONNECT_RETRY)
      break;
  }
  DPRINTLN(msg);
  //Now publish the message
  if(mqtt_client.connected())
  {
    if (mqtt_client.publish(topic.c_str(), msg.c_str(),retain))
    {
      DPRINTLN("message publish success");
      return true;
    }
    else 
    {
      DPRINTLN("message publish fail");
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
    doc["Uptime"] = (unsigned long)millis()/1000;

    if(publishJsonMessage(MQTT_DATA_TOPIC,doc,false))
        PulsesPeriods = 0;
    
    //prepare the json doc for the wifi message
    doc.clear();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["mac"] = WiFi.macAddress();
    doc["version"] = compile_version;
    publishJsonMessage(MQTT_WIFI_TOPIC,doc,false);
    
    publish_tick = false;
  }

  digitalWrite(STATUS_LED_PIN, 0);

  yield();
}
