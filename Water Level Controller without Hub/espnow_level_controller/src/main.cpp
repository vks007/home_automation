/*

// IMPORTANT : Compile it for the device you want, details of which are in Config.h
*/

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
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h" // provides some uitlity functions
#include "espwatchdog.h" // provides capability ot reset the ESP in case it becomes unresponsive
#include <ezLED.h> // provides non blocking blinking of a LED 
// I am using a local copy of this library instead from the std library repo as that has an error in cpp file.
// The author has defined all functions which take a default argumnent again in the cpp file whereas the default argument should only be 
// specified in the decleration and not in the implementation of the function.

// ************ HASH DEFINES *******************
#define VERSION "1.2"
#define MAC_DELIMITER ":" // delimiter with which MAC address is separated
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
#define KEY_LEN  16 // lenght of PMK & LMK key (fixed at 16 for ESP)
#ifndef ESP_OK
  #define ESP_OK 0 // This is defined for ESP32 but not for ESP8266 , so define it
#endif
// API_TIMEOUT is used to determine how long to wait for MQTT connection before restarting the ESP
#define API_TIMEOUT 300 // time in sec to wait for a new message from sensor, beyond which it restarts
#define QUEUE_LENGTH 50 // max no of messages the ESP should queue up before replacing them, limited by amount of free memory
#define HEALTH_INTERVAL 15e3 // interval is millisecs to publish health message for the gateway
// ************ HASH DEFINES *******************
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller

// ************ GLOBAL OBJECTS/VARIABLES *******************
#if USING(EEPROM_STORE)
  const char* ssid = WiFi_SSID; // comes from config.h
#endif
// const char* password = WiFi_SSID_PSWD; // comes from config.h
long last_time = 0; // last time when health check was published
extern "C"
{ 
  #include <lwip/icmp.h> // needed for icmp packet definitions , not sure what was this?
}
ArduinoQueue<espnow_message> structQueue(QUEUE_LENGTH);
ezLED  statusLED(STATUS_LED,CTRL_CATHODE);
watchDog wd = watchDog(API_TIMEOUT); // monitors the MQTT connection, if it is disconnected beyond API_TIMEOUT , it restarts ESP
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

static espnow_message emptyMessage;
espnow_message currentMessage = emptyMessage;

#if defined(ESP32)
esp_now_peer_info_t peerInfo; // This object must be a global object else the setting of peer will fail
#endif
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif

espnow_message myData;
unsigned long start_time;
espnow_device esp_ota_device;
// ************ GLOBAL OBJECTS/VARIABLES *******************
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller



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


void printInitInfo()
{
  DPRINTLN("");
  DPRINTLN("Starting up as a ESPNow Level Controller");
  #if USING(SECURITY)
    DPRINTLN("Security ON");
  #else
    DPRINTLN("Security OFF");
  #endif

}


/*
 * Initializes the ESP with espnow and WiFi client , OTA etc
 */
void setup() {
  // Initialize Serial Monitor , if we're usng pin 3 on ESP8266 for PIR then initialize Serial as Tx only , disable Rx
  // ATTENTION : this is not working as inteded as PIR still holds Rx as LOW while startup and so the ESP doesnt boot
  // As a temp solution , dont turn ON Serial if you're using Rx as PIR PIN
  {DBEGIN(115200);}

  printInitInfo();

  // Set a custom MAC address for the device. This is helpful in cases where you want to replace the actual ESP device in future
  // A custom MAC address will allow all sensors to continue working with the new device and you will not be required to update code on all devices
  #ifdef DEVICE_MAC
  uint8_t customMACAddress[] = DEVICE_MAC; // defined in Config.h
  setCustomMAC(customMACAddress,false);
  #endif

  pinMode(STATUS_LED,OUTPUT);
  pinMode(MOTOR_PIN,OUTPUT);

  String device_name = DEVICE_NAME;
  device_name.replace("_","-");//hostname dont allow underscores or spaces
  WiFi.hostname(device_name.c_str());// Set Hostname.

  // Set the device as a Station and Soft Access Point simultaneously
  // This has to be WIFI_AP_STA and not WIFI_STA, if set to WIFI_STA then it can only receive broadcast messages and stops receiving MAC specific messages.
  // The main problem seems to be caused by the station mode WiFi going into sleep mode while you have no work. This means that it does not listen to receive ESP-Now packets
  // and therefore they are lost. To solve this we will have to force our microcontroller to listen continuously, and this is achieved by turning it into an AP (Access Point)
  //WiFi.mode(WIFI_AP_STA); 
  #if USING(EEPROM_STORE)
    initilizeESP(ssid,MY_ROLE);
  #else
    initilizeESP(DEFAULT_CHANNEL,MY_ROLE,WIFI_AP_STA);
  #endif

  //Init ESP-NOW
  // if (esp_now_init() != ESP_OK) {
  //   DPRINTLN("Error initializing ESP-NOW");
  //   return;
  // }
  
  // esp_now_set_self_role(MY_ROLE);
  
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
  statusLED.turnOFF();
}

bool processMessage(espnow_message msg)
{
  if(msg.intvalue1 == 1) // sensor indicates the tank is full , turn OFF motor
  {
    digitalWrite(MOTOR_PIN,LOW);
    DPRINTLN("Turning motor OFF");
  }
  else // tank is empty , turn ON motor
  {
    digitalWrite(MOTOR_PIN,HIGH);
    DPRINTLN("Turning motor ON");
  }
  return true;
}

/*
 * runs the loop to check for incoming messages in the queue, picks them up and posts them to MQTT
 */
void loop() {
  wd.update(true); //feed the watchdog by calling update
  
    if(!structQueue.isEmpty())
        currentMessage = structQueue.dequeue();
      //else last message content is still there is currentMessage
    
    if(currentMessage != emptyMessage)
    {
      DPRINTFLN("Processing msg id:%lu,type:%d,int1:%d,fval1:%f",currentMessage.message_id,currentMessage.msg_type,currentMessage.intvalue1,currentMessage.floatvalue1);
      if(currentMessage.msg_type == ESPNOW_SENSOR)
      {
        processMessage(currentMessage);
        last_time = millis(); // renew the last_time so that we know we're receiving messages regularly
        if(statusLED.getState() == LED_BLINKING)
        {
          statusLED.cancel(); //cancel blinking to indicate we're good
        }
        statusLED.turnON();// ON indicates we're receiving messages regularly
        currentMessage = emptyMessage;
      }
    }
  if(millis() - last_time > HEALTH_INTERVAL)
  {
    // Blink to warn that a message has not been received since HEALTH_INTERVAL
    if(statusLED.getState() != LED_BLINKING)
    {
      statusLED.blink(500,500);
      DPRINTLN("Blinking...");
    }
  }

  statusLED.loop();

}
