/*
 * This is a temporary water level sensor that I am building in the interim for my house
 * THis senses the water level via water level switches and trasmits the same to the MQTT server on a topic
 * Compiled for Wemos (  I use Wemos PIN names)
*/
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

//#define TESTING_MODE //used to prevent using Rx & Tx as input pins , rather use them as normal serial pins for debugging , comment this out during normal operation
//#define DEBUG //BEAWARE that this statement should be before #include <DebugMacros.h> else the macros wont work as they are based on this #define
#include <DebugMacros.h>

#define SENSOR_OPEN 2
#define SENSOR_CLOSED 3
#define MAX_MQTT_CONNECT_RETRY 4 //max no of retries to connect to MQTT server

#define PIN_LEVEL_25 D2  // Pin at which level 25% sensor is connected
#define PIN_LEVEL_50 D1  // Pin at which level 50% sensor is connected
#define PIN_LEVEL_75 D7  // Pin at which level 75% sensor is connected
#define PIN_LEVEL_100 D6  // Pin at which level 100% sensor is connected


char VERSION[] = "1.0";

//User configuration section
char mqtt_server[16] = "192.168.1.100";//IP address of the MQTT server
const short mqtt_port = 1883;
char mqtt_user[20] = "mqtt_client";//username to connect to MQTT server
char mqtt_pswd[20] = "Mosquitto1@";//password to connect to MQTT server
char mqtt_client_name[20] = "tank_sensor";
char mqtt_topic[50] = "home/tank"; 
char ip_address[16] = "192.168.1.60"; //static IP to be assigned to the chip eg 192.168.1.60
//User configuration section

WiFiClient espClient;
PubSubClient client(espClient);

void setup() 
{
  DBEGIN(115200);
  pinMode(PIN_LEVEL_25, INPUT_PULLUP);
  pinMode(PIN_LEVEL_50, INPUT_PULLUP);
  pinMode(PIN_LEVEL_75, INPUT_PULLUP);
  pinMode(PIN_LEVEL_100, INPUT_PULLUP);

  DPRINTLN("");
  DPRINTLN("Version:" + String(VERSION));
  setupWiFi();
}  

void setupWiFi() 
{
  //Set the IP address (This results in faster connection to WiFi ~ 3sec
  short ip[4];
  char * item = strtok(ip_address, ".");
  char index = 0;
  while (item != NULL) {
    ip[index++] = atoi(item);
    item = strtok(NULL, ".");
  }
  IPAddress esp_ip(ip[0], ip[1], ip[2], ip[3]);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(esp_ip, gateway, subnet);
  WiFi.hostname(mqtt_client_name);// Set Hostname.
  DPRINTLN(".....");
  
  WiFi.mode(WIFI_STA); // Force to station mode because if device was switched off while in access point mode it will start up next time in access point mode.
  WiFi.begin("EAGLE_EXT","HELLO99021");
  for(short i=0;i<3000;i++) //break after 30 sec 3000*10 msec
  {
    if(WiFi.status() != WL_CONNECTED)
      delay(10);//Dont increase this delay, I set it to 500 and it takes a very long time to connect, I think this blocks the execution
    else
    {
      DPRINT("WiFi connected, IP Address:");
      DPRINTLN(WiFi.localIP());
      break;
    }
  }
}

void publishMessage(char msg[10]) {
  // Loop until we're reconnected
  char i = 0;
  while (!client.connected()) 
  {
    i++;
    DPRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_name,mqtt_user,mqtt_pswd)) {
      DPRINTLN("connected");
      char publish_topic[65] = ""; //variable accomodates 50 characters of main topic + 15 char of sub topic
      strcpy(publish_topic,mqtt_topic);
      strcat(publish_topic,"/status"); 
      client.publish(publish_topic, msg);

      DPRINTLN("published messages");
    } 
    else 
    {
      DPRINT("failed, rc=");
      DPRINT(client.state());
      DPRINTLN(" try again in 1 second");
      delay(1000);
    }
    if(i >= MAX_MQTT_CONNECT_RETRY)
      break;
  }
}


void loop() 
{
  if((WiFi.status() == WL_CONNECTED))
  {
    
    client.setServer(mqtt_server, mqtt_port);
    //Publish the message according to the type of message received

    if(digitalRead(PIN_LEVEL_100) == LOW)
      publishMessage("100");
    else if(digitalRead(PIN_LEVEL_75) == LOW)
      publishMessage("75");
    else if(digitalRead(PIN_LEVEL_50) == LOW)
      publishMessage("50");
    else if(digitalRead(PIN_LEVEL_25) == LOW)
      publishMessage("25");
    else
      publishMessage("0");
  }
  delay(2000);
}
