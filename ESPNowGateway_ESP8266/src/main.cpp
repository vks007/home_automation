/*
 * This is a slave (gateway) implemented using a ESP32 which listens to espnow messages from other controllers (masters/sensors) and passes on these messages to MQTT
 * Some points to keep in mind:
 * The gateway connects to an AP (WiFi router) and also listens to espnow messages and this forces the ESP to be on a single channel dictated by the router. Hence the espnow channel is also dictated
 * This means that the masters also have to operate ont he same channel. You can get the channel on the master by scanning the SSID of the router and determining its channel. 
 * Features:
 * - Implements both the WiFi station and ESPNow slave in a single ESP module
 * - Retries MQTT connection a few times before giving up
 * - Supports COMBO role where it can receive and send messages
 * - Publishes initial health message on startup and then a health message at a set interval, stats like msg count, msg rate, queue length, free memory, uptime etc are posted
 * - Supports OTA
 * 
 * TO DO :
 * - encryption isnt working. Even if I change the keys on the master, the slave is able to receieve the messages, so have to debug later
 *  if you can solve the encryption issue, remove it as with excryption , an eSP8266 can only connect to 6 other peers, ESP32 can connect to 10 other
 *  while without encryption they can connect to 20 peers, encryption eg from :https://github.com/espressif/ESP8266_NONOS_SDK/issues/114#issuecomment-383521100
 * - Change the role to COMBO for both slave and Controller so that Slave can also pass on administration messages to the controller.
 * - construct the controller topic from its mac address instead of picking it up from the message id. Instead use message id as a string to identify the device name
 * - Do not pop out the message form the queue in case posting to MQTT isnt successful
 * - Issue with asyncmqttclinet lib : see here : https://github.com/marvinroger/async-mqtt-client/issues/209
 */

// IMPORTANT : Compile it for the device you want, details of which are in Config.h

#define IN_USE == 1
#define NOT_IN_USE == 0
#define USING(feature) 1 feature //macro to check a feature , ref : https://stackoverflow.com/questions/18348625/c-macro-to-enable-and-disable-code-features

//Turn features ON and OFF below
#define SECURITY NOT_IN_USE
/* For now currently turning OFF security as I am not able to make it work. It works even if the keys aren't the same on controller and slave
 * Also I have to find a way to create a list of multiple controllers as with security you haev to register each controller separately
 * See ref code here: https://www.electrosoftcloud.com/en/security-on-your-esp32-with-esp-now/
*/
#define DEBUG (1) //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "Config.h"
#include "secrets.h"
#include <ArduinoJson.h>
#include <ArduinoQueue.h>
#include <ArduinoOTA.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h"
//#include <PubSubClient.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder

#define VERSION "1.3"
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
#define RECONNECT_INTERVAL 5 //retry interval in seconds to reconnect to WiFi & MQTT server
#define QUEUE_LENGTH 50 // This is the no of objects of espnow_messages that you want to buffer, only limited by the amount of free memory you have on the ESP
#define MAX_MESSAGE_LEN 251 // defnies max message length, as the max espnow allows is 250, cant exceed it
#define ESP_OK 0 // This is defined for ESP32 but not for ESP8266 , so define it
#define HEALTH_INTERVAL 10e3 // interval is millisecs to publish health message for the gateway
#define MQTT_LWT_MESSAGE "offline" // The LWT message to post
const char* ssid = WiFi_SSID;
const char* password = WiFi_SSID_PSWD;
long last_time = 0;
bool retry_message = false;
long last_message_count = 0;//stores the last count with which message rate was calculated
long message_count = 0;//keeps track of total no of messages publshed since uptime
String strIP_address = "";//stores the IP address of the ESP
bool startup = true; //flag to indicate startup, is set to false at the end of setup()
char LWT_topic[65] = ""; //variable accomodates 50 characters of main topic + 15 char of sub topic

#define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver

//List of controllers(sensors) who is send messages to this receiver
uint8_t controller_mac[][6] = CONTROLLERS; //from secrets.h
//example entry : 
// uint8_t controller_mac[2][6] = {  
//    {0x4C, 0xF2, 0x32, 0xF0, 0x74, 0x2D} ,
//    {0xC, 0xDD, 0xC2, 0x33, 0x11, 0x98}
// };

#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#else
uint8_t kok[16]= {};//comes from secrets.h
uint8_t key[16] = {};// comes from secrets.h
#endif

ArduinoQueue<espnow_message> structQueue(QUEUE_LENGTH);
//WiFiClient espClient;
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
static espnow_message emptyMessage;
espnow_message currentMessage = emptyMessage;


void connectToWifi() {
  WiFi.config(ESP_IP_ADDRESS, default_gateway, subnet_mask);//from secrets.h
  String device_name = DEVICE_NAME;
  device_name.replace("_","-");//hostname dont allow underscores or spaces
  WiFi.hostname(device_name.c_str());// Set Hostname.
  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  // Set device as a Wi-Fi Station
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
}

void connectToMqtt() {
  DPRINTLN("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  DPRINTLN("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  DPRINTLN("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(RECONNECT_INTERVAL, connectToWifi);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  DPRINTLN("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(RECONNECT_INTERVAL, connectToMqtt);
  }
}

/*
 * publishes a string to MQTT
 */
bool publishToMQTT(const char msg[],const char topic[], bool retain)
{
    if(mqttClient.connected())
    {
      if(mqttClient.publish(topic,2,retain,msg))
      {
        DPRINT("Successfully published:");DPRINTLN(msg);
        return true;
      }
      else
      {
        DPRINT("Failed to Publish:");DPRINTLN(msg);
      }
    }
    DPRINT("Failed to connect to MQTT:");DPRINTLN(msg);
    return false;
}

/*
 * Creates a json string from the espnow message and publishes it to a MQTT queue
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
  strcat(final_publish_topic,"/state");// create a topic to publish the state of the device
  //DPRINTF("publishToMQTT:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
  StaticJsonDocument<MAX_MESSAGE_LEN> msg_json;
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
  
  String str_msg="";
  serializeJson(msg_json,str_msg);
  return publishToMQTT(str_msg.c_str(),final_publish_topic,false);
  //DPRINTF("published message with len:%u\n",measureJson(msg_json));
}

void onMqttConnect(bool sessionPresent) {
  DPRINTLN("Connected to MQTT.");
  DPRINT("Session present: ");DPRINTLN(sessionPresent);
  publishToMQTT("online",LWT_topic,true);
}

/*
 * Callback called on receiving a message. It posts the incoming message in the queue
 */
void OnDataSent(uint8_t *receiver_mac, uint8_t transmissionStatus) {
  if(transmissionStatus == 0) {
    DPRINTLN("Data sent successfully");
  } else {
    DPRINT("Error code: ");DPRINTLN(transmissionStatus);
  }
};

/*
 * Callback called on sending a message.
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  //DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
    if(!structQueue.isFull())
      structQueue.enqueue(msg);
    else
      DPRINTLN("Queue Full");
};

/*
 * creates data for health message and publishes it
 */
void publishHealthMessage()
{
  StaticJsonDocument<MAX_MESSAGE_LEN> msg_json;
  msg_json["uptime"] = getReadableTime(millis());
  msg_json["mem_freeKB"] = serialized(String((float)ESP.getFreeHeap()/ 1024.0,0));//Ref:https://arduinojson.org/v6/how-to/configure-the-serialization-of-floats/
  msg_json["msg_count"] = message_count;
  msg_json["queue_len"] = structQueue.itemCount();
  float message_rate = (message_count - last_message_count)/(float)(HEALTH_INTERVAL/(60*1000));//rate calculated over one minute
  last_message_count = message_count;//reset the count
  msg_json["msg_rate"] = serialized(String(message_rate,1));//format with 1 decimal places, Ref:https://arduinojson.org/v6/how-to/configure-the-serialization-of-floats/

  char publish_topic[65] = "";
  // create a path for this specific device which is of the form MQTT_BASE_TOPIC/<master_id> , master_id is usually the mac address stripped off the colon eg. MQTT_BASE_TOPIC/2CF43220842D
  strcpy(publish_topic,MQTT_TOPIC);
  strcat(publish_topic,"/state");
  String str_msg="";
  serializeJson(msg_json,str_msg);
  publishToMQTT(str_msg.c_str(),publish_topic,false);


  // publish the wifi message , I am publishing this everytime because it also has rssi
  strIP_address = WiFi.localIP().toString();
  StaticJsonDocument<MAX_MESSAGE_LEN> wifi_msg_json;//It is recommended to create a new obj than reuse the earlier one by ArduinoJson
  wifi_msg_json["ip_address"] = strIP_address;
  wifi_msg_json["rssi"] = WiFi.RSSI();
  strcpy(publish_topic,MQTT_TOPIC);
  strcat(publish_topic,"/wifi");
  str_msg="";
  serializeJson(wifi_msg_json,str_msg);
  publishToMQTT(str_msg.c_str(),publish_topic,true);

  //Now publish the init message only if we're starting up
  if(startup)
  {
    // publish the init message
    StaticJsonDocument<MAX_MESSAGE_LEN> init_msg_json;//It is recommended to create a new obj than reuse the earlier one by ArduinoJson
    init_msg_json["version"] = compile_version;
    init_msg_json["tot_memKB"] = (float)ESP.getFlashChipSize() / 1024.0;
    init_msg_json["mac"] = WiFi.macAddress();
    init_msg_json["macAP"] = WiFi.softAPmacAddress();
    strcpy(publish_topic,MQTT_TOPIC);
    strcat(publish_topic,"/init");
    str_msg="";
    serializeJson(init_msg_json,str_msg);
    publishToMQTT(str_msg.c_str(),publish_topic,true);
  }

}

void setup() {
  // Initialize Serial Monitor
  DBEGIN(115200);
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(mqtt_broker, mqtt_port); 
  strcpy(LWT_topic,MQTT_TOPIC);
  strcat(LWT_topic,"/LWT"); 
  mqttClient.setWill(LWT_topic, 2, true, MQTT_LWT_MESSAGE);
  connectToWifi();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  byte channel = wifi_get_channel();
  #if USING(SECURITY)
  // Setting the PMK key
  esp_now_set_kok(kok, 16);
  #endif
  for(byte i = 0;i< sizeof(controller_mac)/6;i++)
  {
    #if USING(SECURITY)
    esp_now_add_peer(controller_mac[i], ESP_NOW_ROLE_CONTROLLER, channel, key, 16);
    esp_now_set_peer_key(controller_mac[i], key, 16);
    #else
    esp_now_add_peer(controller_mac[i], ESP_NOW_ROLE_CONTROLLER, channel, NULL, 16);
    #endif
  }

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
  publishHealthMessage();
  startup = false;
}


void loop() {
  ArduinoOTA.handle();
  
  if(mqttClient.connected())
  {
    if(!structQueue.isEmpty())
      if(!retry_message)
        currentMessage = structQueue.dequeue();
      //else last message content is still there is currentMessage
    
    if(currentMessage != emptyMessage)
    {
      if(publishToMQTT(currentMessage))
      {
        message_count++;
        currentMessage = emptyMessage;
      }
      else
      {
        retry_message = true;
      }//else the same message will be retried the next time
    }
  }
  
  if(millis() - last_time > HEALTH_INTERVAL)
  {
    //collect health params and publish the health message
    publishHealthMessage();
    last_time = millis();
  }
}
