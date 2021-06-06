/*
 * ver 1.0.0 - fully working version. publishes messages to MQTT when data changes
 * ver 1.1.0 - sleep functionality , ESP sleeps if there is no data change for a certain period and wakes up on recieving pulses from the meter
 * TO DO LIST
 * - store data into RTC memory - see if you really want this to happen as power cycling the ESP will not reset stats then, will have to find an alternative to resetting
 * REFERENCES:
 * https://github.com/esp8266/Arduino/tree/master/libraries/esp8266/examples/LowPowerDemo
 * https://gitlab.com/diy_bloke/verydeepsleep_mqtt.ino/blob/master/VeryDeepSleep_MQTT.ino
 * 
 * LEARNINGS
 * I had mislabeled WAKEUP pin and PULSE pin which was causing the ESP to reset after 7 sec of sleepby WDT. Pl be aware of the pins
 * If i connect the USB and separate power both to the ESP then i get a few pulses on the PULSE pin due to noise, disconnect USB and it goes away
 * so dont work with USB connected.
 * CAUTION: Make sure the MQTT connect and publish code is not within the callback. It creates a lot of issues with reconnecting to MQTT after
 * the expiry of keepalive timeout.
 */

//  ***************** HASH DEFINES ************ THIS SHOULD BE THE FIRST SECTION IN THE CODE BEFORE ANY INCLUDES *******************************
#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
//#define USE_WEBSOCKETS
#define USE_OTA
#define FLOW_METER_TANK
//  ***************** HASH DEFINES *******************************************


#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>

//common files from /include
#include "secrets.h" 
#include "version.h" 
#include "myutils.h" //common utilities
#include "Debugutils.h" 
#include "ESPOTA.h" //OTA capability
#include "websocket_log.h"

ESP8266WebServer server;

#include "flow_meter_config.h"

#define VERSION "1.2.1"
const char compile_version[] = VERSION __DATE__ " " __TIME__;

const char ssid[] = SSID1; //SSID of WiFi to connect to
const char ssid_pswd[] = SSID1_PSWD; //Password of WiFi to connect to

const char* deviceName = "flow_meter_tank";

//GPIO pin defn
#define PULSE_PIN D2
#define WAKEUP_PIN D1

//config params
#define REPORT_INTERVAL 5 // interval in seconds at which the message is posted to MQTT when the ESP is awake , cannot be greater than IDLE_TIME
#define SENSOR_UPDATE_INTERVAL 1 //interval in seconds at which sensor(flow) values are updated
#define IDLE_TIME 300 //300 //idle time in sec beyond which the ESP goes to sleep , to be woken up only by a pulse from the meter

#define DEBOUNCE_INTERVAL 10 //debouncing time in ms for interrupts
#define PULSE_PER_LIT 255 //no of pulses the meter counts for 1 lit of water , adjust this for each water meter after calibiration
#define MAX_MQTT_CONNECT_RETRY 3
#define MAX_JSON_LEN  200 //max no of characters in a json doc
#define STATE_TOPIC "state"
#define WIFI_TOPIC "wifi"
#define DEBUG_TOPIC "debug"

volatile unsigned int pulses = 0;
volatile unsigned int last_pulses = 0;
volatile unsigned int total_pulses = 0;
volatile float flow_rate = 0.0;//flow rate is per minute
volatile float total_volume = 0.0;//stores the total volume of liquid since start
unsigned long last_publish_time = 0;//stores the no of ms since a message was last published , used to sleep the ESP beyond a certain value
unsigned long sleep_time_start = 0;
unsigned long sleep_time = 0;
unsigned long up_time = RTCmillis();
unsigned awake_count = 0;

#ifdef KLIT
  const float factor = 0.001;
#else  //LIT - default
  const float factor = 1.0;
#endif

os_timer_t publish_timer;//timer to publish values to MQTT
os_timer_t sensor_timer; // timer to calculate latest sensor values

bool publish_tick = true;// flag to keep track of when to enable publishing of message, initial value of true publishes a message on startup

volatile unsigned long lastMicros;
IPAddress esp_ip ;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void IRAM_ATTR pulseHandler() {
  if((long)(micros() - lastMicros) >= DEBOUNCE_INTERVAL * 1000) { //note DEBOUNCE_INTERVAL is in ms , so multiply by 1000 for microsec
    pulses += 1;
    lastMicros = micros();
  }
}

void timerInit(void) {
  os_timer_setfn(&publish_timer, publish_timer_callback, NULL);
  os_timer_arm(&publish_timer, 1000 * REPORT_INTERVAL, true);
  os_timer_setfn(&sensor_timer, sensor_timer_callback, NULL);
  os_timer_arm(&sensor_timer, 1000 * SENSOR_UPDATE_INTERVAL, true);
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
  if(WiFi.status() == WL_CONNECTED)
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

// connection to WiFi is not critical at this stage, it can happen in the background
// If WiFi is not connected, messages will be pooled till it is available anyway
// commenting out the wait for connection below does not work, i never get a WiFi connection , will debug later
// maybe i can use the PersWiFiManager library from here: https://github.com/r-downing/PersWiFiManager  
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
      SETUP_OTA();
      mqtt_client.setServer(MQTT_SERVER1, MQTT_PORT1);
      // dont connect to MQTT here, connect when required during publishign of the message
      //connectMQTT();
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
  WS_BROADCAST_TXT(msg);
  DPRINT(topic);DPRINT("-->");DPRINT(msg);
  
  bool result = false;
  //Now publish the message
  if(connectMQTT())
  {
    //WS_BROADCAST_TXT("Connected to MQTT");
    if (mqtt_client.publish(topic.c_str(), msg.c_str(),retain))
    {
      DPRINTLN(" - OK");
      WS_BROADCAST_TXT(" - OK\n");
      result = true;
    }
    else 
    {
      DPRINTLN(" - FAIL");
      WS_BROADCAST_TXT(" - FAIL\n");
      result = false;
    }
  }
  else
  {
    WS_BROADCAST_TXT("Not Connected to MQTT\n");
    DPRINTLN("Failed to establish MQTT connection...");
    result = false;
  }
  
  mqtt_client.disconnect(); //close MQTT connection cleanly
  return result;
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
    sleep_time_start = RTCmillis(); //record the time before sleeping
    wifi_fpm_do_sleep(0xFFFFFFF);  // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(10);  // it goes to sleep during this delay() and waits for an interrupt
    DPRINTLN(F("Woke up!"));  // the interrupt callback hits before this is executed*/
    //sometimes the above statement gets printed before actually sleeping off, doesnt happen all the time though
    //But the code works as expected, the ESP sleeps after printing Woke up
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
      // If you want to round off to 2 decimal places see here : https://arduinojson.org/v6/how-to/configure-the-serialization-of-floats/
      // I think it would not be a good idea to round off in a sensor, this should be done in the consumer of this message
      doc["flow_rate"] = (float)flow_rate;
      doc["flow_unit"] = (factor == 1?"lit/min":"Klit/min");
      doc["total_volume"] = (float)total_volume; 
      doc["vol_unit"] = (factor == 1?"lit":"Klit");
      up_time = RTCmillis();
      doc["up_time"] = (unsigned long)up_time/1000;
      doc["sleep_time"] = (unsigned long)sleep_time/1000;
      doc["awake_count"] = awake_count;
      publishJsonMessage(MQTT_BASE_TOPIC STATE_TOPIC,doc,false);
    }

    if(type == 'A' || type == 'D')
    {
      #ifdef DEBUG
        //prepare the debug message 
        doc.clear();
        doc["total_pulses"] = total_pulses;
        publishJsonMessage(MQTT_BASE_TOPIC DEBUG_TOPIC,doc,false);
      #endif
    }

    //prepare wifi info message only if the IP has changed, else it was published during setup()
    // This will be the case the first time the control comes here as esp_ip will be blank
    if(esp_ip != WiFi.localIP())
    {
      esp_ip = WiFi.localIP();
      if((type == 'W' || type == 'A'))
        publishWiFimsg();
    }
}

/*
 * calculates the flow rate
 */
float calculate_flow_rate()
{
  return (float)pulses/(PULSE_PER_LIT*SENSOR_UPDATE_INTERVAL)*60.0*factor;
}

/*
 * calculates the flow rate
 */
float calculate_total_volume()
{
  return (float)total_pulses/PULSE_PER_LIT*factor;
}

void sensor_timer_callback(void *pArg) {
/*
  flow_rate = calculate_flow_rate();
  total_pulses += pulses;
  pulses = 0;//reset pulses as this is needed to calculate rate per SENSOR_UPDATE_INTERVAL only
  total_volume = calculate_total_volume();
*/
  total_pulses += pulses;
  DPRINTLN(total_pulses);
  flow_rate = (float)pulses/(PULSE_PER_LIT*SENSOR_UPDATE_INTERVAL)*60.0*factor;
  total_volume = (float)total_pulses/PULSE_PER_LIT*factor;
  pulses = 0;//reset pulses as this is needed to calculate rate per REPORT_INTERVAL only
}


//system_get_rst_info ()

void publish_timer_callback(void *pArg) {
  //only publish a message if the values have changed
  if(last_pulses != pulses)
  {
    last_pulses = pulses;
    publish_tick = true;
  }
}

void setup() {
  awake_count = 1; // set the awake count to 1
  DBEGIN(115200);
  delay(1);
  DPRINTLN("");
  DPRINTLN(compile_version);
//  pinMode(LED_BUILTIN, OUTPUT);

  setupWiFi();

  pinMode(WAKEUP_PIN, INPUT_PULLUP);//INPUT_PULLUP
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(PULSE_PIN, pulseHandler, RISING);

  WS_SERVER_SETUP();
  server.begin();
  WS_SETUP();

  timerInit();
  publishMessages('A');//this is only published once unless the IP changes in between
}

void loop() 
{
  WS_LOOP();
  server.handleClient();
  
  if(publish_tick)
  {
    publishMessages('A');
  }
  unsigned long idle_time = millis() - last_publish_time;
  if(idle_time >= IDLE_TIME * 1000)
  {
    WS_BROADCAST_TXT("going to sleep after being idle\n");
    DPRINT("going to sleep after being idle for :"); DPRINT(idle_time/1000);DPRINTLN(" sec");DFLUSH();
//    digitalWrite(LED_BUILTIN,HIGH);
    light_sleep();
//    digitalWrite(LED_BUILTIN,LOW);
    //now that we're awake , add the sleep time to the previous value
    sleep_time += (RTCmillis() - sleep_time_start);
    awake_count++; //we're awake one more time , so increment the counter
    last_publish_time = millis();
    delay(10);
    setupWiFi();
  }
 
  HANDLE_OTA();

  yield();
}
