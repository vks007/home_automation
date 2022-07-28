/*
 * This is a slave (gateway) implemented using a ESP32 which listens to espnow messages from other controllers (masters/sensors) and passes on these messages to MQTT
 * While this slave is aon ESP32 , the masters can be on ESP32 or ESP8266.
 * Some points to keep in mind:
 * The gateway connects to an AP (WiFi router) and also listens to espnow messages and this forces the ESP to be on a single channel dictated by the router. Hence the espnow channel is also dictated
 * This means that the masters also have to operate ont he same channel. You can get the channel on the master by scanning the SSID of the router and determining its channel. 
 * 
 * 
*/

#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Config.h"
#include "secrets.h"
#include <ArduinoJson.h>

// Use only core 1 for your tasks , leave core 0 for WiFi on multicore uC
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define VERSION "1.0"
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string

#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#define MAX_MQTT_CONNECT_RETRY 2 //max no of retries to connect to MQTT server
#define QUEUE_LENGTH 10
#define MAX_MESSAGE_LEN 251 // defnies max message length, as the max espnow allows is 250, cant exceed it

// Replace with your network credentials (STATION)
const char* ssid = WiFi_SSID;
const char* password = WiFi_SSID_PSWD;
long lastMsg = 0;

// Datatypes in Arduino : https://www.tutorialspoint.com/arduino/arduino_data_types.htm
//For a struct of size 96 bytes , the program adds 10 bytes and so the final size being transmitted is 116 bytes
typedef struct struct_message{
  char master_id[13];//contains the mac address without colon (12 chars + null)
  unsigned int message_id;
  bool bytevalue1;
  bool bytevalue2;
  bool bytevalue3;
  bool bytevalue4;
  float floatvalue1;
  float floatvalue2;
  float floatvalue3;
  float floatvalue4;
  char chardata1[10];
  char chardata2[10];
}struct_message;

QueueHandle_t structQueue; //queue to store all incoming messages before they are picked up to be posted to MQTT

WiFiClient espClient;
PubSubClient client(espClient);

bool publishToMQTT(struct_message msg) {
  // Loop until we're reconnected
  char publish_topic[65] = ""; //variable accomodates 50 characters of main topic + 15 char of sub topic
  strcpy(publish_topic,MQTT_TOPIC);
  strcat(publish_topic,"/LWT"); 
  char i = 0;
  while (!client.connected()) 
  {
    i++;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(DEVICE_NAME,mqtt_uname,mqtt_pswd,publish_topic,0,true,"offline")) {//credentials come from secrets.h
      Serial.println("connected");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      vTaskDelay(1000 / portTICK_PERIOD_MS);//delay for 1 sec
    }
    if(i >= MAX_MQTT_CONNECT_RETRY)
      break;
  }//end while

  //Publish the messages now
  if(client.connected())
  {
    char final_publish_topic[65] = "";
    // create a path for this specific device which is of the form MQTT_BASE_TOPIC/<master_id> , master_id is usually the mac address stripped off the colon eg. MQTT_BASE_TOPIC/2CF43220842D
    strcpy(publish_topic,MQTT_BASE_TOPIC);
    strcat(publish_topic,"/");
    //Serial.print("master_id:");Serial.println(msg.master_id);
    strcat(publish_topic,msg.master_id); // from here on all messages would be published within this topic specific for this device
    strcpy(final_publish_topic,publish_topic);
    strcat(final_publish_topic,"/state");// create a topic to publish the state of the device
    //Serial.print("Final topic:");Serial.println(final_publish_topic);
    Serial.printf("publishToMQTT:%u,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.bytevalue1,msg.bytevalue2,msg.bytevalue3,msg.bytevalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
    
    StaticJsonDocument<MAX_MESSAGE_LEN> msg_json;
    msg_json["id"] = msg.message_id;
    msg_json["device"] = msg.master_id;
    msg_json["bval1"] = msg.bytevalue1;
    msg_json["bval2"] = msg.bytevalue2;
    msg_json["bval3"] = msg.bytevalue3;
    msg_json["bval4"] = msg.bytevalue4;
    msg_json["fval1"] = msg.floatvalue1;
    msg_json["fval2"] = msg.floatvalue2;
    msg_json["fval3"] = msg.floatvalue3;
    msg_json["fval4"] = msg.floatvalue4;
    msg_json["char1"] = msg.chardata1;
    msg_json["char2"] = msg.chardata2;
    
    String str_msg="";
    serializeJson(msg_json,str_msg);
    client.publish(final_publish_topic,str_msg.c_str(),true);
    //Serial.printf("published messages with len:%u\n",measureJson(msg_json));
    return true;
  }
  return false;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  //Serial.printf("OnDataRecv:%u,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.bytevalue1,msg.bytevalue2,msg.bytevalue3,msg.bytevalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  
  if(xQueueSend(structQueue, &msg, portMAX_DELAY) != pdTRUE){
    Serial.println("Queue Full");
  }
   
  // One tick delay (15ms) in between reads for stability
  vTaskDelay(portTICK_PERIOD_MS);
}

void TaskPostToMQTT(void * pvParameters) {
   (void) pvParameters;
   struct_message currentMessage;
   while(true)
   {
     if (xQueueReceive(structQueue,&currentMessage, portMAX_DELAY) == pdPASS) {
           publishToMQTT(currentMessage);
           // TO DO , see if the message was posted successfully if not then 
           // disable retrieving new messages in the next loop until we run out of queue space
           // then start retriving messages as we're not left with any choice
      }
      else
      {
        Serial.printf("Queue empty\n");
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
   }
}

void setup() {

  structQueue = xQueueCreate(QUEUE_LENGTH, // Queue length
                               sizeof(struct struct_message) // Queue item size
                            );
  if (structQueue != NULL) {
    // Create task that consumes the queue if it was created.
    xTaskCreate(TaskPostToMQTT,"PostToMQTT",4*1024,NULL,1,NULL);
  }
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  
  // Set device as a Wi-Fi Station
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  if((WiFi.status() == WL_CONNECTED))
  {
    client.setServer(mqtt_broker, mqtt_port);// from secrets.h
  }
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}


void loop() {
  client.loop();
  
}
