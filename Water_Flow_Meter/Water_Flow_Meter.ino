/*
 * ver 1.0.0 - fully working version. publishes messages to MQTT when data changes
 * ver 1.1.0 - sleep functionality , ESP sleeps if there is no data change for a certain period and wakes up on recieving pulses from the meter
 * TO DO LIST
 * - store data into RTC memory
 * REFERENCES:
 * https://github.com/esp8266/Arduino/tree/master/libraries/esp8266/examples/LowPowerDemo
 * CAUTION: Make sure the MQTT connect and publish code is not within the callback. It creates a lot of issues with reconnecting to MQTT after
 * the expiry of keepalive timeout.
 * TO DO : See the code here to sleep and do MQTT publish, maybe this will avoid the crash problem I am facing with this program
 * https://gitlab.com/diy_bloke/verydeepsleep_mqtt.ino/blob/master/VeryDeepSleep_MQTT.ino
 */
#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#include "Debugutils.h" 

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>

//common files from /include
#include "secrets.h" 
#include "version.h" 
#include "myutils.h" //common utilities
#include "ESPOTA.h" //OTA capability
#include "websocket_log.h"
ESP8266WebServer server;

#define FLOW_METER_TANK
#include "flow_meter_config.h"

#define VERSION "1.2.0"
const char compile_version[] = VERSION __DATE__ " " __TIME__;

const char ssid[] = SSID1; //SSID of WiFi to connect to
const char ssid_pswd[] = SSID1_PSWD; //Password of WiFi to connect to

const char* deviceName = "flow_meter_tank";

//GPIO pin defn
#define PULSE_PIN D1
#define WAKEUP_PIN D2

//config params
#define REPORT_INTERVAL 5 // interval in seconds at which the message is posted to MQTT when the ESP is awake , cannot be greater than IDLE_TIME
#define IDLE_TIME 120 //300 //idle time in sec beyond which the ESP goes to sleep , to be woken up only by a pulse from the meter

#define DEBOUNCE_INTERVAL 10 //debouncing time in ms for interrupts
#define PULSE_PER_LIT 255 //no of pulses the meter counts for 1 lit of water , adjust this for each water meter after calibiration
#define MAX_MQTT_CONNECT_RETRY 3
#define MAX_JSON_LEN  100 //max no of characters in a json doc
#define STATE_TOPIC "state"
#define WIFI_TOPIC "wifi"
#define DEBUG_TOPIC "debug"

volatile unsigned int Pulses = 0;
volatile unsigned int TotalPulses = 0;
volatile float flow_rate = 0.0;//flow rate is per minute
volatile float total_volume = 0.0;//stores the total volume of liquid since start
volatile float last_volume = 0.0;//stores the last value of total_volume 
unsigned long last_publish_time = 0;//stores the no of ms since a message was last published , used to sleep the ESP beyond a certain value

#ifdef KLIT
  const float factor = 0.001;
#else  //LIT - default
  const float factor = 1.0;
#endif

os_timer_t publish_timer;
bool publish_tick = false;//to enable publishing of message on startup
volatile unsigned long lastMicros;
IPAddress esp_ip ;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void ICACHE_RAM_ATTR pulseHandler() {
  if((long)(micros() - lastMicros) >= DEBOUNCE_INTERVAL * 1000) {
    Pulses += 1;
    lastMicros = micros();
  }
}

void timerInit(void) {
  os_timer_setfn(&publish_timer, timerCallback, NULL);
  os_timer_arm(&publish_timer, 1000 * REPORT_INTERVAL, true);
}

boolean connectMQTT()
{
  if(!mqtt_client.connected()){
    // Loop until we're reconnected
    char i = MAX_MQTT_CONNECT_RETRY;
    while (!mqtt_client.connect("flow_meter",MQTT_USER1,MQTT_PSWD1) && i > 0)
    {
      DPRINTLN("Attempting MQTT connection...");
      delay(1000);
      i--;
    }
  }
  return mqtt_client.connected();
}

void setupWiFi() 
{
  if(WiFi.status() == WL_CONNECTED && mqtt_client.connected())
  {
    DPRINTLN("Already connected");
    return;
  }
  else
    os_timer_disarm(&publish_timer); //disable the time else it might try to publish messages and fail
  
  WiFi.config(ESP_IP_ADDRESS, GATEWAY1, SUBNET1);
  WiFi.hostname(DEVICE_NAME);
//  hostname += String(ESP.getChipId(), HEX);
  DPRINTLN("Hostname: " + WiFi.hostname());
  DPRINT("Connecting to ");DPRINT(ssid);

  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid,ssid_pswd);
  
  for(short i=0;i<6000;i++) //break after 60 sec 6000*10 msec
  {
    if(WiFi.status() != WL_CONNECTED)
    {
//      if(i%50 == 0)//print a progress dot every 500 ms
        DPRINT(">");
      delay(500);
    }
    else
    {
      DPRINTLN("");
      DPRINT("WiFi connected, IP Address:");
      DPRINTLN(WiFi.localIP());
      esp_ip = WiFi.localIP();
      setupOTA();
      mqtt_client.setServer(MQTT_SERVER1, MQTT_PORT1);
      connectMQTT();
      break;
    }
  }
  os_timer_arm(&publish_timer, 1000 * REPORT_INTERVAL, true); //re-enable the timer
}


/*
 * Function to connect to MQTT if not connected, and then publish the message
 */
bool publishMessage(String topic, String msg,bool retain) {
  if(WiFi.status() != WL_CONNECTED)
      return false;
  connectMQTT();

  webSocket.broadcastTXT(msg);
  DPRINT(topic);DPRINT("-->");DPRINT(msg);
  //Now publish the message
  if(mqtt_client.connected())
  {
    //webSocket.broadcastTXT("Connected to MQTT");
    if (mqtt_client.publish(topic.c_str(), msg.c_str(),retain))
    {
      DPRINTLN(" - OK");
      webSocket.broadcastTXT(" - OK\n");
      return true;
    }
    else 
    {
      DPRINTLN(" - FAIL");
      webSocket.broadcastTXT(" - FAIL\n");
      return false;
    }
  }
  else
  {
    webSocket.broadcastTXT("Not Connected to MQTT\n");
    DPRINTLN("Failed to establish MQTT connection...");
  }
  
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
  doc["ip_address"] = esp_ip.toString();
  doc["mac"] = WiFi.macAddress();
  doc["version"] = compile_version;
  publishJsonMessage(MQTT_BASE_TOPIC WIFI_TOPIC,doc,true);
}

void light_sleep(){
    DPRINTLN("CPU going to sleep, pull WAKE_UP_PIN low to wake it");DFLUSH();
    WiFi.mode(WIFI_OFF);  // you must turn the modem off; using disconnect won't work
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    gpio_pin_wakeup_enable(GPIO_ID_PIN(WAKEUP_PIN), GPIO_PIN_INTR_LOLEVEL);// only LOLEVEL or HILEVEL interrupts work, no edge, that's a CPU limitation
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xFFFFFFF);  // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(10);  // it goes to sleep during this delay() and waits for an interrupt
    DPRINTLN(F("Woke up!"));  // the interrupt callback hits before this is executed*/
 }

/*
 * Publishes messages to MQTT
 * char -> A - All messages , D - only debug messages , W - WiFi message
 */
void publishMessages(char type)
{
    //Set the publish flag to false & reset time irrespective of if the msg publish is a success
    //If it fails , it will anyway retry after the next interval
    publish_tick = false;
    last_publish_time = millis();

    //prepare state message
    StaticJsonDocument<MAX_JSON_LEN> doc;
    if(type == 'A')
    {
      doc["flow_rate"] = flow_rate;
      doc["flow_unit"] = (factor == 1?"lit/min":"Klit/min");
      doc["total_volume"] = total_volume;
      doc["vol_unit"] = (factor == 1?"lit":"Klit");
      doc["uptime"] = (unsigned long)millis()/1000;
      publishJsonMessage(MQTT_BASE_TOPIC STATE_TOPIC,doc,false);
    }

    if(type == 'A' || type == 'D')
    {
      #ifdef DEBUG
        //prepare the debug message 
        doc.clear();
        doc["TotalPulses"] = TotalPulses;
        publishJsonMessage(MQTT_BASE_TOPIC DEBUG_TOPIC,doc,false);
      #endif
    }

    //prepare wifi info message only if the IP has changed, else it was published during setup()
    if((esp_ip != WiFi.localIP()) && (type == 'W' || type == 'A'))//means the IP add has changed or we want to publish WiFi message forcefully
      publishWiFimsg();
}

//system_get_rst_info ()
void timerCallback(void *pArg) {
  TotalPulses += Pulses;
  flow_rate = (float)Pulses/(PULSE_PER_LIT*REPORT_INTERVAL)*60.0*factor;
  last_volume = total_volume; //save last value
  total_volume = (float)TotalPulses/PULSE_PER_LIT*factor;

  //only publish a message if the values have changed, if flow rate changed, total volume would also change so no 
  //need to check both
  if(Pulses > 0)
  {
    Pulses = 0;//reset pulses as this is needed to calculate rate per REPORT_INTERVAL only
    //publishMessages('A');
    publish_tick = true;
  }
//  else {
//    DPRINTLN("no change in pulses");
//    //publishMessages('D');
//  }
}

void setup() {
  DBEGIN(115200);
  delay(1);
  DPRINTLN("");
  DPRINTLN(compile_version);

  setupWiFi();

  pinMode(WAKEUP_PIN, INPUT_PULLUP);//INPUT_PULLUP
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(PULSE_PIN, pulseHandler, RISING);

  server.on("/log",[](){
  server.send_P(200, "text/html", webpage);
  });
  server.begin();
  ws_setup();

  timerInit();
//  publishMessages('W');//this is only published once unless the IP changes in between
}

void loop() 
{
  ws_loop();
  server.handleClient();
  
  if(publish_tick)
  {
    publishMessages('A');
  }
  unsigned long idle_time = millis() - last_publish_time;
  if(idle_time >= IDLE_TIME * 1000)
  {
    webSocket.broadcastTXT("going to sleep after being idle\n");
    DPRINT("going to sleep after being idle for :"); DPRINT(idle_time/1000);DPRINTLN(" sec");DFLUSH();
    light_sleep();
    
    last_publish_time = millis();
    delay(10);
    setupWiFi();
  }
 
  ArduinoOTA.handle();

  yield();
}
