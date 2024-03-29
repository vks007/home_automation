/*
 * This is a slave (gateway) implemented using a ESP32 which listens to espnow messages from other controllers (masters/sensors) and passes on these messages to MQTT
 * Some points to keep in mind:
 * The gateway connects to an AP (WiFi router) and also listens to espnow messages and this forces the ESP to be on a single channel dictated by the router. Hence the espnow channel is also dictated
 * This means that the masters also have to operate ont he same channel. You can get the channel on the master by scanning the SSID of the router and determining its channel. 
 * 
 * 
 * WiFi address:60:01:94:5C:A1:8D
 * SoftAP address:62:01:94:5C:A1:8D
 * TO DO :
 * 1. create an array of controllers to receive messages from them 
 * 2. encryption isnt working. Even if I change the keys on the master, the slave is able to receieve the messages, so have to debug later
 * - if you can solve the encryption issue, remove it as with excryption , an eSP8266 can only connect to 6 other peers, ESP32 can connect to 10 other
 * - while without encryption they can connect to 20 peers, encryption eg from :https://github.com/espressif/ESP8266_NONOS_SDK/issues/114#issuecomment-383521100
 * 3. Change the role to COMBO for both slave and Controller so that Slave can also pass on administration messages to the controller.
 * 4. construct the controller topic from its mac address instead of picking it up from the message id. Instead use message id as a string to identify the device name
 * - assign a static IP address to the ESP than a dynamic one
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <PubSubClient.h>
#include "Config.h"
#include "secrets.h"
#include <ArduinoJson.h>
#include <ArduinoQueue.h>
#include <ArduinoOTA.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h"

#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#define VERSION "1.0"
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
#define MAX_MQTT_CONNECT_RETRY 2 //max no of retries to connect to MQTT server
#define QUEUE_LENGTH 50
#define MAX_MESSAGE_LEN 251 // defnies max message length, as the max espnow allows is 250, cant exceed it
#define ESP_OK 0 // This is defined for ESP32 but not for ESP8266 , so define it
#define HEALTH_INTERVAL 30e3 // interval is millisecs to publish health message for the gateway
const char* ssid = WiFi_SSID;
const char* password = WiFi_SSID_PSWD;
long last_time = 0;
long message_count = 0;//keeps track of total no of messages publshed since uptime
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
String strIP_address = "";//stores the IP address of the ESP

ArduinoQueue<espnow_message> structQueue(QUEUE_LENGTH);
WiFiClient espClient;
PubSubClient client(espClient);

//uint8_t controller_mac[6][6] = {{0xA0, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA},{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
uint8_t controller_mac2[6] = {0x2C, 0xF4, 0x32, 0x20, 0x84, 0x2D};//2C:F4:32:20:84:2D - wemos
uint8_t controller_mac[6] = {0xBC, 0xDD, 0xC2, 0x83, 0x01, 0x98};//BC:DD:C2:83:01:98 - temperature sensor ESP12

/*
 * connects to MQTT server , publishes LWT message as "online" every time it connects
 */
void reconnectMQTT()
{
  byte i = 0;
  while (!client.connected()) 
  {
    i++;
    DPRINT("Attempting MQTT connection...");
    // Attempt to connect
    // publishes the LWT message ("online") to the topic,If the this device disconnects from the broker ungracefully then the broker automatically posts the "offline" message on the LWT topic
    // so that all connected clients know that this device has gone offline
    char publish_topic[65] = ""; //variable accomodates 50 characters of main topic + 15 char of sub topic
    strcpy(publish_topic,MQTT_TOPIC);
    strcat(publish_topic,"/LWT"); 
    if (client.connect(DEVICE_NAME,mqtt_uname,mqtt_pswd,publish_topic,0,true,"offline")) {//credentials come from secrets.h
      DPRINTLN("connected");
      client.publish(publish_topic,"online",true);
    }
    else 
    {
      DPRINT("failed, rc=");
      DPRINT(client.state());
      DPRINTLN(" try again in 1 second");
      delay(1000);//delay for 1 sec
    }
    if(i >= MAX_MQTT_CONNECT_RETRY)
      break;
  }//end while
  
}

/*
 * publishes a string to MQTT
 */
bool publishToMQTT(const char msg[],const char topic[], bool retain)
{
    if(!client.connected())
      reconnectMQTT();
    if(client.connected())
      return client.publish(topic,msg,retain);
    else
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
  //DPRINT("master_id:");DPRINTLN(msg.master_id);
  strcat(publish_topic,msg.device_name); // from here on all messages would be published within this topic specific for this device
  strcpy(final_publish_topic,publish_topic);
  strcat(final_publish_topic,"/state");// create a topic to publish the state of the device
  //DPRINT("Final topic:");DPRINTLN(final_publish_topic);
  Serial.printf("publishToMQTT:%u,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
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
  //Serial.printf("published message with len:%u\n",measureJson(msg_json));
}


/*
 * Callback called on receiving a message. It posts the incoming message in the queue
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  //Serial.printf("OnDataRecv:%u,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
    if(!structQueue.isFull())
      structQueue.enqueue(msg);
    else
      DPRINTLN("Queue Full");
};

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
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

  if((WiFi.status() == WL_CONNECTED))
  {
    client.setServer(mqtt_broker, mqtt_port);// from secrets.h
  }
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }
  // Setting the PMK key
  esp_now_set_kok(kok, 16);
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  esp_now_add_peer(controller_mac, ESP_NOW_ROLE_CONTROLLER, wifi_get_channel(), key, 16);
  esp_now_set_peer_key(controller_mac, key, 16);
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
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();


}

/*
 * creates data for health message and publishes it
 */
void publishHealthMessage()
{
  StaticJsonDocument<MAX_MESSAGE_LEN> msg_json;
  msg_json["uptime"] = getReadableTime(millis());
  msg_json["mem_freeKB"] = (float)ESP.getFreeHeap()/ 1024.0;
  msg_json["tot_memKB"] = (float)ESP.getFlashChipSize() / 1024.0;
  msg_json["msg_count"] = message_count;
  msg_json["queue_len"] = structQueue.itemCount();
  
  char publish_topic[65] = "";
  // create a path for this specific device which is of the form MQTT_BASE_TOPIC/<master_id> , master_id is usually the mac address stripped off the colon eg. MQTT_BASE_TOPIC/2CF43220842D
  strcpy(publish_topic,MQTT_TOPIC);
  strcat(publish_topic,"/state");
 String str_msg="";
  serializeJson(msg_json,str_msg);
  publishToMQTT(str_msg.c_str(),publish_topic,false);

  //Now publish the wifi message if the IP has changed
  if(strIP_address != WiFi.localIP().toString())
  {
    strIP_address = WiFi.localIP().toString();
    StaticJsonDocument<MAX_MESSAGE_LEN> wifi_msg_json;//It is recommended to create a new obj than reuse the earlier one by ArduinoJson
    wifi_msg_json["ip_address"] = strIP_address;
    wifi_msg_json["mac"] = WiFi.macAddress();
    wifi_msg_json["macAP"] = WiFi.softAPmacAddress();
    strcpy(publish_topic,MQTT_TOPIC);
    strcat(publish_topic,"/wifi");
  
    str_msg="";
    serializeJson(wifi_msg_json,str_msg);
    publishToMQTT(str_msg.c_str(),publish_topic,true);
  }
}

void loop() {
  client.loop();
  ArduinoOTA.handle();
  
  if (!structQueue.isEmpty()){
    espnow_message currentMessage;
    currentMessage = structQueue.dequeue();
    publishToMQTT(currentMessage);
    message_count++;
    // TO DO , see if the message was posted successfully if not then 
    // disable retrieving new messages in the next loop until we run out of queue space
    // then start retriving messages as we're not left with any choice
  }
  
  if(millis() - last_time > HEALTH_INTERVAL)
  {
    //collect health params and publish the health message
    publishHealthMessage();
    last_time = millis();
  }
}
