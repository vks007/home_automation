/*
 * This is a slave (gateway) implemented using a ESP8266 which listens to espnow messages from other controllers (masters/sensors) and passes on these messages to MQTT
 * Some points to keep in mind:
 * The gateway connects to an AP (WiFi router) and also listens to espnow messages and this forces the ESP to be on a single channel dictated by the router. Hence the espnow channel is also dictated
 * This means that the masters also have to operate ont he same channel. You can get the channel on the master by scanning the SSID of the router and determining its channel. 
 * Features:
 * - Implements both the WiFi station and ESPNow slave in a single ESP module
 * - Retries MQTT connection a few times before giving up
 * - Publishes initial health message on startup and then a health message at a set interval, stats like msg count, msg rate, queue length, free memory, uptime etc are posted
 * - Supports OTA
 * - DONE - Do not pop out the message form the queue in case posting to MQTT isnt successful
 * 
 * TO DO :
 * - replace EEPROM library with LittleFS as EEPROM is deprecated
 * - implement passing of OTA trigger params like , timeperiod, interval via web page rather than hard coded
 * - implement websockets to indicate progress of OTA trigger and its result
 * - implement encryption, instead of using inbuilt one you can use Chacha for encryption, see this library for details : https://github.com/eccnil/ESPNow2Mqtt
 * - encryption isnt working. Even if I change the keys on the master to random values, the slave is able to receieve the messages, so have to debug later
 *  if you can solve the encryption issue, remove it as with excryption , an eSP8266 can only connect to 6 other peers, ESP32 can connect to 10 other
 *  while without encryption they can connect to 20 peers, encryption eg from :https://github.com/espressif/ESP8266_NONOS_SDK/issues/114#issuecomment-383521100
 * - Change the role to COMBO for both slave and Controller so that Slave can also pass on administration messages to the controller.
 * - construct the controller topic from its mac address instead of picking it up from the message id. Instead use message id as a string to identify the device name
 * - Introduce a status LED for MQTT connection status
 * - Implement a restart of ESP after configurable interval if the connection to MQTT is not restored
 * - To Support COMBO role where it can receive and send messages
 * - Support automatic pairing of peers , see here for ideas : https://randomnerdtutorials.com/esp-now-auto-pairing-esp32-esp8266/
 */

// IMPORTANT : Compile it for the device you want, details of which are in Config.h
/* For now currently turning OFF security as I am not able to make it work. It works even if the keys aren't the same on controller and slave
 * Also I have to find a way to create a list of multiple controllers as with security you haev to register each controller separately
 * See ref code here: https://www.electrosoftcloud.com/en/security-on-your-esp32-with-esp-now/
*/
//#define MQTT_MAX_PACKET_SIZE 2048

#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

#include "Config.h" // defines all Config parameters. set your values before compiling
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#include <espnow.h> // provides espnow capabilities for ESP8266
#include "secrets.h" // provides all passwords and sensitive info
#include <ArduinoJson.h> // provides json capabilities for messages published to MQTT
#include <ArduinoQueue.h> // provide Queue management
#include <ArduinoOTA.h> 
#include "espnowMessage.h" // for struct of espnow message
#include <PubSubClient.h> // library for MQTT
#include <HARestAPI.h> // library to make rest API calls to Home Assistant (included locally)
#include <Pinger.h> // used for pinging a server (gatewya) in case of connection issues with MQTT
#include "myutils.h" // provides some uitlity functions
#include "espwatchdog.h" // provides capability ot reset the ESP in case it becomes unresponsive
#include "pir_sensor.h" // provides capability to attach a PIR sensor to the ESP
#include <ezLED.h> // provides non blocking blinking of a LED 
// I am using a local copy of this library instead from the std library repo as that has an error in cpp file.
// The author has defined all functions which take a default argumnent again in the cpp file whereas the default argument should only be 
// specified in the decleration and not in the implementation of the function.

// ************ HASH DEFINES *******************
#define VERSION "1.2"
#define MAC_DELIMITER ":" // delimiter with which MAC address is separated
#define OTA_TRIGGER_DURATION 10 // time in secs for which trigger the ESPNOW OTA messages
#define OTA_MSG_INTERVAL  50 //interval in ms at which to repeat OTA messages
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
#define KEY_LEN  16 // lenght of PMK & LMK key (fixed at 16 for ESP)
#define MQTT_RETRY_INTERVAL 5000 //MQTT server connection retry interval in milliseconds
#define STATE_TOPIC "/state" // end path of topic to publish state of the messages
#define OTA_TOPIC "/ota" // end path of topic to publish ota messages
#define ERROR_TOPIC "/error" // end path of topic to publish error messages
#define QUEUE_LENGTH 10 // max no of messages the ESP should queue up before replacing them, limited by amount of free memory
#define MAX_MSG_BUFFER_SIZE 512 // max size of the MQTT packet buffer, it includes topic name+payload+header bytes, set your payload max lenn accordingly using MAX_MESSAGE_LEN below 
#define MAX_MESSAGE_LEN 412 // defines max message length of payload message. Included space for 100 bytes for topic name + header
#define HEALTH_INTERVAL 30e3 // interval is millisecs to publish health message for the gateway
#ifndef ESP_OK
  #define ESP_OK 0 // This is defined for ESP32 but not for ESP8266 , so define it
#endif
#define MAX_MQTT_PUBLISH_FAILURES 5 //max no of times MQTT publishing can fail even though the MQTT client was connected beyond which ESP restart happens
// ************ HASH DEFINES *******************

// ************ GLOBAL OBJECTS/VARIABLES *******************
const char* ssid = WiFi_SSID; // comes from config.h
const char* password = WiFi_SSID_PSWD; // comes from config.h
long last_time = 0; // last time when health check was published
bool retry_message = false; // flag to indicate to retry sending of message in case of failure
long last_message_count = 0;//stores the last count with which message rate was calculated
long lastReconnectAttempt = 0; // Keeps track of the last time an attempt was made to connect to MQTT
short mqtt_publish_fails = 0; //tracks the no of times MQTT publishing has failed even though the MQTT client was connected
bool initilized = false; // flag to track if initialisation of the ESP has finished. At present it only handles tracking of the "init" message published on startup
extern "C"
{ 
  #include <lwip/icmp.h> // needed for icmp packet definitions , not sure what was this?
}
Pinger pinger;
ezLED  statusLED(STATUS_LED);
watchDog MQTT_wd = watchDog(); // monitors the MQTT connection, if it is disconnected beyond API_TIMEOUT , it restarts ESP
//List of controllers(sensors) who will send messages to this receiver
uint8_t controller_mac[][6] = CONTROLLERS; //from secrets.h
/* example entry : 
uint8_t controller_mac[2][6] = {  
   {0x4C, 0xF2, 0x32, 0xF0, 0x74, 0x2D} ,
   {0xC, 0xDD, 0xC2, 0x33, 0x11, 0x98}
}; */
// Define security keys, these are random hex numbers
uint8_t kok[KEY_LEN]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[KEY_LEN] = LMK_KEY_STR;// comes from secrets.h

ArduinoQueue<espnow_message> structQueue(QUEUE_LENGTH);
WiFiClient espClient;
PubSubClient client(espClient);
static espnow_message emptyMessage;
espnow_message currentMessage = emptyMessage;

WiFiClient haClient;
HARestAPI ha_api(haClient);

#if(USING(ESPNOW_OTA_SERVER))
  AsyncWebServer server(80);
  const char* input_param_mac = "input_mac";
#endif


#if USING(MOTION_SENSOR)
pir_sensor motion_sensor(PIR_PIN,MOTION_ON_DURATION);
#endif
// HTML web page to handle web server requests for ESPNOW OTA functionality
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESPNOW OTA</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Times New Roman; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem; color: #FF0000;}
  </style>
  </head><body>
  <form action="/espnowota">
    Enter the MAC Address: <input type="text" name="input_mac"><br>
    Total Duration(s): <input type="text" name="input_duration" value="10"><br>
    OTA Request Interval(ms): <input type="text" name="input_interval"  value="50"><br>
    <input type="submit" value="Send OTA Request">
  </form><br>
</body></html>)rawliteral";

#if defined(ESP32)
esp_now_peer_info_t peerInfo; // This object must be a global object else the setting of peer will fail
#endif
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif

espnow_message myData;
unsigned long start_time;
unsigned long last_msg_sent_time;
espnow_device esp_ota_device;
typedef struct gateway_stats
{
  unsigned long msg_count;
  unsigned long failed_msg_count;
  unsigned long cmnd_msg_count;
  float msg_rate;
  float free_mem_KB;// free memory in KB
  unsigned long uptime; // uptime in minutes
  int rssi;
  String str_mac;
  String str_macAP;
  short wifi_channel;
  short queue_length;
  String last_published_msg;
}gateway_stats;
gateway_stats gateway;

// ************ GLOBAL OBJECTS/VARIABLES *******************
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller


/*
 * connects to MQTT server , publishes LWT message as "online" every time it connects
 * waits for a time defined by MQTT_RETRY_INTERVAL before reconnecting again
 * I had issues with MQTT nto able to connect at times and so I included pinger to ping the gateway but then the issue hasnt happened for a long time now
 */
bool reconnectMQTT()
{
  if (!client.connected())
  {
    long now = millis();
    if ((now - lastReconnectAttempt > MQTT_RETRY_INTERVAL) || lastReconnectAttempt == 0) 
    {
      lastReconnectAttempt = now;
      DPRINTLN("Attempting MQTT connection...");
      // publishes the LWT message ("online") to the topic,If the this device disconnects from the broker ungracefully then the broker automatically posts the "offline" message on the LWT topic
      // so that all connected clients know that this device has gone offline
      char publish_topic[65] = ""; //variable accomodates 50 characters of main topic + 15 char of sub topic
      strcpy(publish_topic,MQTT_TOPIC);
      strcat(publish_topic,"/LWT"); 
      if (client.connect(DEVICE_NAME,mqtt_uname,mqtt_pswd,publish_topic,0,true,"offline")) {//credentials come from secrets.h
        DPRINTLN("MQTT connected");
        client.publish(publish_topic,"online",true);
        if(statusLED.getState() == LED_BLINKING)
          statusLED.cancel();
        return true;
      }
    }
    if(statusLED.getState() == LED_IDLE)
      statusLED.blink(1000, 500);
    return false;
  }
  if(statusLED.getState() == LED_BLINKING)
    statusLED.cancel();
  return true;
}

/*
 * Publishes a string message to MQTT , returns true if message was published successfully else false
 * IMPORTANT : Encountered a strange issue where the device was more than MAX_DEVICE_ID_LEN characters long, this caused the MQTT to return success for publish method but the message wasnt published
 * In the next step the MQTT conneciton was lost and it was re-established. So be careful of the length of the device name from the client side. Sample message below:  terrace_flow_meter is more than MAX_DEVICE_ID_LEN
 *  const char* json_msg = R"({"gateway":"gateway-ff","type":0,"mac":"62:01:94:FF:01:08","id":2,"device":"terrace_flow_meter","ival1":0,"ival2":0,"ival3":0,"ival4":0,"fval1":0,"fval2":0,"fval3":0,"fval4":0,"char1":"","char2":"1.0 Jan 11 2025"})";
 * This bug doesnt make sense as this finally is only a string in this method and the message was within MQTT_MAX_PACKET_SIZE (256). Didnt have time to debug this further so left it
 * The maximum length of a message that can be published on MQTT depends on the MQTT broker and the client library you are using. For the ESP8266 with the PubSubClient library, the default maximum message size is 256 bytes
 * #define MQTT_MAX_PACKET_SIZE 256 can be changed to #define MQTT_MAX_PACKET_SIZE 1024
 * Also be aware of : MAX_MSG_BUFFER_SIZE : This is the max size of the MQTT packet buffer, it includes topic name+payload+header bytes, set your payload max lenn accordingly using MAX_MESSAGE_LEN below
 * MAX_MESSAGE_LEN : defines max message length of payload message. Included space for 100 bytes for topic name + header
 */
bool publishToMQTT(const char msg[],const char topic[], bool retain)
{
  if(strlen(msg) > MQTT_MAX_PACKET_SIZE)
  {
    DPRINTFLN("publishToMQTT- Failed,Message too long:%u",strlen(msg));
    return false;
  }
  DPRINTFLN("msg:%s",msg);
  if(client.connected())
  {
    if(client.publish(topic,msg,retain))
    {
      DPRINTLN("publishToMQTT-Published.");
      if(mqtt_publish_fails>0)
        mqtt_publish_fails = 0;//reset the counter
      return true;
    }
    else
    {
      mqtt_publish_fails++;
      if(mqtt_publish_fails >= MAX_MQTT_PUBLISH_FAILURES)
      {
        DPRINTLN("publishToMQTT-Failed max allowed times. triggering ESP restart");
        MQTT_wd.update(false,true); // ask the watchdog to restart the ESP
      }
      else
      {
        // see if disconnecting the client will solve the issue
        client.disconnect();
        DPRINTLN("publishToMQTT-Failed, willfully disconnected MQTT.");
      }
    }
  }
  else
    DPRINTLN("Publish failed - MQTT not connected");
  return false;
}

/*
 * Creates a json string from the espnow message and publishes it to a MQTT queue. Truncates the message if it exceeds MAX_MESSAGE_LEN
 * Returns true if message was published successfully else false
 */
bool publishToMQTT(espnow_message msg) {
  char final_publish_topic[65] = "";
  char publish_topic[65] = "";
  // create a path for this specific device which is of the form MQTT_BASE_TOPIC/<master_id> , master_id is usually the mac address stripped off the colon eg. MQTT_BASE_TOPIC/2CF43220842D
  strcpy(publish_topic,MQTT_BASE_TOPIC);
  strcat(publish_topic,"/");
  strcat(publish_topic,msg.device_name); // from here on all messages would be published within this topic specific for this device
  strcpy(final_publish_topic,publish_topic);
  if(msg.msg_type == ESPNOW_SENSOR)
    strcat(final_publish_topic,STATE_TOPIC);// create a topic to publish the state of the device
  else if(msg.msg_type == ESPNOW_OTA)
    strcat(final_publish_topic,OTA_TOPIC);// create a topic to publish the ota state of the device
  else // unknown message type , publish this on the error topic
    strcat(final_publish_topic,ERROR_TOPIC);// create a topic to publish error

//  DPRINTF("publishToMQTT:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
  StaticJsonDocument<MAX_MESSAGE_LEN> msg_json;
  msg_json["gateway"] = WiFi.hostname();
  msg_json["type"] = msg.msg_type;
  msg_json["mac"] = msg.sender_mac;
  msg_json["id"] = msg.message_id;
  msg_json["device"] = msg.device_name;
  msg_json["ival1"] = msg.intvalue1;
  msg_json["ival2"] = msg.intvalue2;
  msg_json["ival3"] = msg.intvalue3;
  msg_json["ival4"] = msg.intvalue4;
  msg_json["fval1"] = msg.floatvalue1;
  msg_json["fval2"] = msg.floatvalue2;
  msg_json["fval3"] = msg.floatvalue3;
  msg_json["fval4"] = msg.floatvalue4;
  msg_json["char1"] = msg.chardata1;
  msg_json["char2"] = msg.chardata2;

  // Serialize the JSON document to a string
  String str_msg;
  serializeJson(msg_json, str_msg);

  // Check if the length exceeds MAX_MESSAGE_LEN
  if (str_msg.length() >= MAX_MESSAGE_LEN) {
    // Calculate the excess length
    int excess_length = str_msg.length() - MAX_MESSAGE_LEN;

    // Truncate the char2 field by the excess length if it is within the limit of 64 characters
    if (excess_length < 64) {
      msg.chardata2[strlen(msg.chardata2) - excess_length] = '\0'; // Truncate the char2 field
      excess_length = 0;
    }
    else {
      // Truncate the char2 entirely and add a null terminator
      msg.chardata2[0] = '\0';
      excess_length = excess_length - 64;
    }
    if (excess_length < 64) {
      msg.chardata1[strlen(msg.chardata2) - excess_length] = '\0'; // Truncate the char2 field
      excess_length = 0;
    }
    // not proceeding with further truncation as there is no possibility of the string exceeding further

    // Re-serialize the JSON document to a string
    msg_json["char2"] = msg.chardata2;
    str_msg = "";
    serializeJson(msg_json, str_msg);
  }

  DPRINTF("Going to publish message with len:%u\n", str_msg.length());
  return publishToMQTT(str_msg.c_str(), final_publish_topic, false);
}

#if USING(MOTION_SENSOR)
/*
 * Creates a message string for motion ON message and publishes it to a MQTT queue
 * Returns true if message was published successfully else false
 */
bool publishMotionMsgToMQTT(const char topic_name[],const char state[4]) {
  char publish_topic[65] = "";
  // create a path for this specific device which is of the form MQTT_BASE_TOPIC/<master_id> , master_id is usually the mac address stripped off the colon eg. MQTT_BASE_TOPIC/2CF43220842D
  strcpy(publish_topic,MQTT_TOPIC);
  strcat(publish_topic,"/");
  strcat(publish_topic,topic_name);
  strcat(publish_topic,"/state");
  return publishToMQTT(state,publish_topic,false);
}
#endif

/*
 * Callback when data is sent , It sets the bResultReady flag to true on successful delivery of message
 * The flag is set to false in the main loop where data is sent and then the code waits to see if it gets set to true, if not it retires to send
 */
esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) {
  deliverySuccess = status;
  //DPRINT("OnDataSent:Last Packet delivery status:\t");
  //DPRINTLN(status == 0 ? "Success" : "Fail");
  bResultReady = true;
});

/*
 * Callback called on sending a message.
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.msg_type,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
    if(!structQueue.isFull())
      structQueue.enqueue(msg);
    else
      DPRINTLN("Queue Full");
};

/*
 * updates the gateway statistics , called before publishing a health message
 */
void update_gateway_stats()
{
  gateway.queue_length = structQueue.itemCount();
  gateway.rssi = WiFi.RSSI();
  gateway.str_mac = WiFi.macAddress();
  gateway.str_macAP = WiFi.softAPmacAddress();
  gateway.uptime = millis()/6000; //publish uptime in minutes
  gateway.wifi_channel = WiFi.channel();
  gateway.msg_rate = (gateway.msg_count - last_message_count)/(float)(HEALTH_INTERVAL/(60*1000));//rate calculated over one minute
  last_message_count = gateway.msg_count;//reset the count
  gateway.free_mem_KB = ESP.getFreeHeap()/ 1024.0;
}

/*
 * creates data for health message and publishes it
 * takes bool param init , if true then publishes the startup message else publishes the health check message
 */
bool publishHealthMessage(bool init=false)
{
  char publish_topic[65] = "";
  String str_msg="";
  update_gateway_stats();

  if(init)
  {
     // publish the init message
    StaticJsonDocument<MAX_MESSAGE_LEN> init_msg_json;//It is recommended to create a new obj than reuse the earlier one by ArduinoJson
    init_msg_json["version"] = compile_version;
    init_msg_json["tot_memKB"] = (float)ESP.getFlashChipSize() / 1024.0;
    init_msg_json["mac"] = gateway.str_mac;
    init_msg_json["macAP"] = gateway.str_macAP;
    init_msg_json["wifiChannel"] = gateway.wifi_channel;
    strcpy(publish_topic,MQTT_TOPIC);
    strcat(publish_topic,"/init");
    str_msg="";
    serializeJson(init_msg_json,str_msg);
    return publishToMQTT(str_msg.c_str(),publish_topic,true);
  }
  else
  {
    StaticJsonDocument<MAX_MESSAGE_LEN> msg_json;
    msg_json["uptime"] = gateway.uptime; // uptime in minutes
    msg_json["mem_freeKB"] = serialized(String((float)gateway.free_mem_KB,0));//Ref:https://arduinojson.org/v6/how-to/configure-the-serialization-of-floats/
    msg_json["msg_count"] = gateway.msg_count;
    msg_json["queue_len"] = gateway.queue_length;
    last_message_count = gateway.msg_count;//reset the count
    msg_json["msg_rate"] = serialized(String(gateway.msg_rate,1));//format with 1 decimal places, Ref:https://arduinojson.org/v6/how-to/configure-the-serialization-of-floats/

    // create a path for this specific device which is of the form MQTT_BASE_TOPIC/<master_id> , master_id is usually the mac address stripped off the colon eg. MQTT_BASE_TOPIC/2CF43220842D
    strcpy(publish_topic,MQTT_TOPIC);
    strcat(publish_topic,"/state");
    serializeJson(msg_json,str_msg);
    if(publishToMQTT(str_msg.c_str(),publish_topic,false))
    {
      // publish the wifi message , I am publishing this everytime because it also has rssi
      String strIP_address = WiFi.localIP().toString();
      StaticJsonDocument<MAX_MESSAGE_LEN> wifi_msg_json;//It is recommended to create a new obj than reuse the earlier one by ArduinoJson
      wifi_msg_json["ip_address"] = strIP_address;
      wifi_msg_json["rssi"] = WiFi.RSSI();
      strcpy(publish_topic,MQTT_TOPIC);
      strcat(publish_topic,"/wifi");
      str_msg="";
      serializeJson(wifi_msg_json,str_msg);
      return publishToMQTT(str_msg.c_str(),publish_topic,true);
   }
  }
  return false;//control will never come here
}


void printInitInfo()
{
  DPRINTLN("Starting up as a ESPNow Gateway");
  #if USING(MOTION_SENSOR)
    DPRINTLN("Motion sensor ON");
  #else
    DPRINTLN("Motion sensor OFF");
  #endif
  #if USING(SECURITY)
    DPRINTLN("Security ON");
  #else
    DPRINTLN("Security OFF");
  #endif

}

#if(USING(ESPNOW_OTA_SERVER))
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

/*
 * Adds ESP MAC as a peer to which OTA messages need to be send and sets the relevant flags. The loop() then takes care of sending the messages to this device
 * param: peerAddress: address of the peer as an uint8 array
 * param: duration : duration in sec for which to keep sending this message
 * param: stop_on_delivery : flag to indicate it it should stop sending messages once it gets the delivery confirmation
 */
void prepare_for_OTA(uint8_t peerAddress[])
{
//  espnow_message myData;
  #if(USING(SECURITY))
    refreshPeer(peerAddress, key,RECEIVER_ROLE);
  #else
    refreshPeer(peerAddress, NULL,RECEIVER_ROLE);
  #endif
  // If devicename is not given then generate one from MAC address stripping off the colon
  #ifdef DEVICE_NAME
    strcpy(myData.device_name,DEVICE_NAME);
  #else
    String wifiMacString = WiFi.macAddress();
    wifiMacString.replace(":","");
    snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
  #endif
  myData.msg_type = ESPNOW_OTA;
  esp_ota_device.ota_mode = true; // set the ota flag which will trigger the OTA flow in loop()
  esp_ota_device.ota_done = false; // reset the success flag
  start_time = millis(); // start the timer
  //DPRINTLN("Device prepared for sending OTA messages:%02X:%02X:%02X:%02X:%02X:%02X ",peerAddress[0],peerAddress[1],peerAddress[2],peerAddress[3],peerAddress[4],peerAddress[5]);
  DPRINTLN("Device prepared for sending OTA messages");
}

void config_webserver()
{
  // Handles root page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Handles OTA request for an ESPNOW device
  server.on("/espnowota", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String mac_addr , response_msg;
    // OTA can be triggered only for one device at a time, reject this request if one is already in progress
    if(!esp_ota_device.ota_mode)
    {
      if (request->hasParam(input_param_mac)) {
        mac_addr = request->getParam(input_param_mac)->value();
        if(validate_MAC(mac_addr.c_str()))
        {
          DPRINTFLN("MAC %s is valid",mac_addr.c_str());
          mac_addr.replace(MAC_DELIMITER,"");
          MAC_to_array(mac_addr.c_str(),esp_ota_device.mac,6);
          // now extract the other params
          String str_temp;
          if (request->hasParam("input_duration")) {
            str_temp = request->getParam("input_duration")->value();
            if(isNumeric(str_temp))
              esp_ota_device.duration = (short)str_temp.toInt();
            else
              esp_ota_device.duration = OTA_TRIGGER_DURATION;
          }
          if (request->hasParam("input_interval")) {
            str_temp = request->getParam("input_interval")->value();
            if(isNumeric(str_temp))
              esp_ota_device.interval = (short)str_temp.toInt();
            else
              esp_ota_device.interval = OTA_MSG_INTERVAL;
          }
          prepare_for_OTA(esp_ota_device.mac);
          response_msg = "ESPNOW OTA req sent to device: ";
        }
        else
        {
          response_msg = "MAC address provided in invalid : ";
          DPRINTFLN("MAC %s is invalid",mac_addr.c_str());
        }
      }
      else 
      {
        mac_addr = "none";
      }
    }
    else
    {
      char buffer[18];
      sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",esp_ota_device.mac[0],esp_ota_device.mac[1],esp_ota_device.mac[2],esp_ota_device.mac[3],esp_ota_device.mac[4],esp_ota_device.mac[5] );
      response_msg = "OTA already in progress for device: " + String(buffer);
      DPRINTLN(response_msg);
    }
    request->send(200, "text/html", response_msg + mac_addr + " <br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();  
}
#endif


/*
 * Initializes the ESP with espnow and WiFi client , OTA etc
 */
void setup() {
  // Initialize Serial Monitor , if we're usng pin 3 on ESP8266 for PIR then initialize Serial as Tx only , disable Rx
  // ATTENTION : this is not working as inteded as PIR still holds Rx as LOW while startup and so the ESP doesnt boot
  // As a temp solution , dont turn ON Serial if you're using Rx as PIR PIN
  #if USING(MOTION_SENSOR)
  if(PIR_PIN == 3)
    {DBEGIN(115200, SERIAL_8N1, SERIAL_TX_ONLY);}
  else
    {DBEGIN(115200);}
  #else
    {DBEGIN(115200);}
  #endif

  DPRINTFLN("Value of MQTT_MAX_PACKET_SIZE is %u",MQTT_MAX_PACKET_SIZE);
  printInitInfo();

  // Set the device as a Station and Soft Access Point simultaneously
  // This has to be WIFI_AP_STA and not WIFI_STA, if set to WIFI_STA then it can only receive broadcast messages and stops receiving MAC specific messages.
  // The main problem seems to be caused by the station mode WiFi going into sleep mode while you have no work. This means that it does not listen to receive ESP-Now packets
  // and therefore they are lost. To solve this we will have to force our microcontroller to listen continuously, and this is achieved by turning it into an AP (Access Point)
  WiFi.mode(WIFI_AP_STA); 

  // Set a custom MAC address for the device. This is helpful in cases where you want to replace the actual ESP device in future
  // A custom MAC address will allow all sensors to continue working with the new device and you will not be required to update code on all devices
  uint8_t customMACAddress[] = DEVICE_MAC; // defined in Config.h
  setCustomMAC(customMACAddress,false);

  pinMode(STATUS_LED,OUTPUT);
  WiFi.config(ESP_IP_ADDRESS, default_gateway, subnet_mask);//from secrets.h
  String device_name = DEVICE_NAME;
  device_name.replace("_","-");//hostname dont allow underscores or spaces
  WiFi.hostname(device_name.c_str());// Set Hostname.

  // Connect to the WiFi as a station device
  WiFi.begin(ssid, password);
  DPRINTLN("Setting as a Wi-Fi Station..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DPRINT(".");
  }
  DPRINT("Station IP Address: ");
  DPRINTLN(WiFi.localIP());
  DPRINT("Wi-Fi Channel: ");
  DPRINTLN(WiFi.channel());
  WiFi.setAutoReconnect(true);

  if((WiFi.status() == WL_CONNECTED))
  {
    client.setServer(mqtt_broker, mqtt_port);// from secrets.h
  }
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_set_self_role(MY_ROLE);
  #if USING(SECURITY)
  // Setting the PMK key
  esp_now_set_kok(kok, KEY_LEN);
  byte channel = wifi_get_channel();
  // Add each controller who is expected to send a message to this gateway
  for(byte i = 0;i< sizeof(controller_mac)/6;i++)
  {
    esp_now_add_peer(controller_mac[i], RECEIVER_ROLE, channel, key, KEY_LEN);
    DPRINTFLN("Added controller :%02X:%02X:%02X:%02X:%02X:%02X",controller_mac[i][0],controller_mac[i][1],controller_mac[i][2],controller_mac[i][3],controller_mac[i][4],controller_mac[i][5] );
  }
   #endif

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  DPRINT("WiFi address:");DPRINTLN(WiFi.macAddress());
  DPRINT("SoftAP address:");DPRINTLN(WiFi.softAPmacAddress());

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    DPRINTLN("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    DPRINTLN("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DPRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DPRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      DPRINTLN("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DPRINTLN("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DPRINTLN("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DPRINTLN("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DPRINTLN("End Failed");
    }
  });
  ArduinoOTA.begin();
  #if(USING(ESPNOW_OTA_SERVER))
  config_webserver();
  #endif
  //set MQTT buffer size
  if(client.getBufferSize() < MAX_MSG_BUFFER_SIZE)
  {
    if(!client.setBufferSize(MAX_MSG_BUFFER_SIZE))
      {DPRINTFLN("Failed to set MQTT buffer size to %d. Any messages over %d will fail to publish",MAX_MSG_BUFFER_SIZE,MQTT_MAX_PACKET_SIZE);}
    else
      {DPRINTFLN("Set MQTT buffer size to %d",MAX_MSG_BUFFER_SIZE);}
  }
  
  reconnectMQTT(); //connect to MQTT before publishing the startup message
  if(publishHealthMessage(true)) //publish the startup message
    initilized = true;
  #if USING(MOTION_SENSOR)
  if(!motion_sensor.begin(MOTION_SENSOR_NAME)) //initialize the motion sensor
    DPRINTLN("Failed to initialize motion sensor");
  #endif
  
  ha_api.setHAServer(ha_url, ha_port);
  ha_api.setHAPassword(ha_token);

}

/*
 * processes the command type messages which are calls to Home Assistant APIs with actions
 * at present always returns true, which means does not retry if failed.
 */
bool process_command_message(espnow_message &msg)
{
  if(msg.msg_type != ESPNOW_COMMAND)
    return false;
  char msg_data[sizeof(msg.chardata1)*2] = "";
  strcpy(msg_data,msg.chardata1);
  DPRINTLN(msg_data);
  strcat(msg_data,msg.chardata2);
  DPRINTLN(msg_data);
  char delim[] = "|";
  char *ptr = strtok(msg_data, delim);
  String domain = String(ptr);
  ptr = strtok(NULL, delim);
  String action = String(ptr);
  ptr = strtok(NULL, delim);
  String data = String(ptr);

  String serviceURL = "/api/services/";
  serviceURL = serviceURL + domain + "/" + action;
  DPRINTLN(serviceURL);
  DPRINTLN(data);
  ha_api.sendPostHA(serviceURL,data);
  return true;
}


#if USING(MOTION_SENSOR)
/*
 * determines the state of motion sensor and publishes a message accordingly
 */
void update_motion_sensor()
{
  short motion_state = motion_sensor.update();
  if(motion_state == 1)
  {
    DPRINTLN("Motion detected as ON");
    publishMotionMsgToMQTT(motion_sensor.getSensorName(),"on");
  }
  else if(motion_state == 2)
  {
    DPRINTLN("Motion detected as OFF");
    publishMotionMsgToMQTT(motion_sensor.getSensorName(),"off");
  }
}
#endif

void do_ota_server()
{
  #if(USING(ESPNOW_OTA_SERVER))
    if(esp_ota_device.ota_mode && !esp_ota_device.ota_done)
    {
      if(millis() < (start_time + esp_ota_device.duration*1000))
      {
        if((millis() - last_msg_sent_time) > esp_ota_device.interval)
        {
          myData.message_id = millis();
          bool result = sendESPnowMessage(&myData,esp_ota_device.mac,0,false);
          last_msg_sent_time = millis();
          //yield();// trying this out to see if the MQTT disconect bug gets solved
          if(result)
          {
            //DPRINTFLN("%lu: sent OTA message",myData.message_id);
          }
          else
          {
            DPRINTFLN("%lu: OTA message sending failed",myData.message_id);}
        }
      }
      else
      {
        esp_ota_device.ota_mode = false;
        esp_ota_device.ota_done = true;
        delete_peer(esp_ota_device.mac);// now that we're done with OTA , delete the peer
        DPRINTLN("OTA timed out without acknowlegdment from device");
      }
    }
  #endif

}

void do_health_check()
{
  // try to publish health message irrespective of the state of espnow messages
  if(millis() - last_time > HEALTH_INTERVAL)
  {
    if(initilized)
    {
      //publish the health message
      publishHealthMessage();
    }
    else
    // It might be possible when the ESP comes up MQTT is down, in that case an init message will not get published in setup()
    // The statement below will check and publish the same , only once
    {
      if(publishHealthMessage(true))
        initilized = true;
    }
    last_time = millis(); // This is reset irrespective of a successful publish else the main loop will continously try to publish this message
  }
}

void do_queue_check()
{
  if(structQueue.isFull())
  {
    DPRINTLN("Queue Full, restarting ESP to see if the issue goes away, messages will be lost");
    MQTT_wd.update(false,true); // ask the watchdog to restart the ESP
  }
}

/*
 * runs the loop to check for incoming messages in the queue, picks them up and posts them to MQTT
 */
void loop() {
  //check for MQTT connection
  if (!client.connected()) 
  {
    reconnectMQTT(); // Attempt to reconnect
  } else {
    client.loop(); // Client connected
  }
  MQTT_wd.update(client.connected()); //feed the watchdog by calling update
  
  #if USING(MOTION_SENSOR)
    update_motion_sensor();
  #endif

  ArduinoOTA.handle();
  if(client.connected())
  {
    if(!structQueue.isEmpty())
      if(!retry_message)
        currentMessage = structQueue.dequeue();
      //else last message content is still there is currentMessage
    
    if(currentMessage != emptyMessage)
    {
      DPRINTFLN("Processing msg:%lu,%d",currentMessage.message_id,currentMessage.msg_type);
      if(currentMessage.msg_type == ESPNOW_SENSOR)
      {
        if(publishToMQTT(currentMessage))
        {
          gateway.msg_count++;
          currentMessage = emptyMessage;
        }
        else
        {
          retry_message = true;
        }//else the same message will be retried the next time
      }
      else if(currentMessage.msg_type == ESPNOW_COMMAND)
      {
        if(process_command_message(currentMessage))
        {
          gateway.msg_count++;
          gateway.cmnd_msg_count++;
          currentMessage = emptyMessage;
        }
        else
        {
          retry_message = true;
        }//else the same message will be retried the next time
      }
      #if(USING(ESPNOW_OTA_SERVER))
      else if(currentMessage.msg_type == ESPNOW_OTA)
      {
        if(esp_ota_device.ota_mode)
        {
          char buffer[18];
          sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X",esp_ota_device.mac[0],esp_ota_device.mac[1],esp_ota_device.mac[2],esp_ota_device.mac[3],esp_ota_device.mac[4],esp_ota_device.mac[5] );
          if(xstrcmp(buffer,currentMessage.sender_mac))
          {
            esp_ota_device.ota_mode = false;
            esp_ota_device.ota_done = true;
            delete_peer(esp_ota_device.mac);// now that we're done with OTA , delete the peer
            DPRINTLN("OTA successfuly completed for device:");
          }
          else
            {DPRINTFLN("MAC address of OTA device did not match. Target mac: %s , MAC in msg: %s",buffer,currentMessage.sender_mac);}
        }
        else
          DPRINTLN("OTA ack message received when not in OTA mode, ignoring...");
        // pubish the OTA message to MQTT
        publishToMQTT(currentMessage);
        gateway.msg_count++;
        currentMessage = emptyMessage;
      }
      #endif
      else
      {
        DPRINTLN("message received without msg_type set, ignoring...");
        gateway.msg_count++;
        currentMessage = emptyMessage;
      }
    }
  }
  do_health_check();
  do_queue_check();
  statusLED.loop();
  do_ota_server();

}
