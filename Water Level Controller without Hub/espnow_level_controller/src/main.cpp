/*
This is the code for water level controller. It is based on ESPNOW and also supports WiFi STA mode.
If WiFi isnt present then it continues in the espnow mode and does normal operation.
If Wifi is present it creates aa webpage on the root address
// IMPORTANT : Set the device parameters in Config.h and then compile it
It is not tested with ESP32 as yet
*/

#include <Arduino.h>
#if defined(ESP8266)
  /* ESP8266 Dependencies */
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  /* ESP32 Dependencies */
  #include <WiFi.h>
  #include <AsyncTCP.h>
#endif
#include <ESPDash.h>
#include <ESPAsyncWebServer.h>

#include "Config.h" // defines all Config parameters. set your values before compiling
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#include <espnow.h> // provides espnow capabilities for ESP8266
#include "secrets.h" // provides all passwords and sensitive info
#include <ArduinoJson.h> // provides json capabilities for messages published to MQTT
#include <ArduinoQueue.h> // provide Queue management
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h" // provides some uitlity functions
#include "Debounce.h" // provided debounce library for buttons
#include <ezLED.h> // provides non blocking blinking of a LED 
// I am using a local copy of this library instead from the std library repo as that has an error in cpp file.
// The author has defined all functions which take a default argumnent again in the cpp file whereas the default argument should only be 
// specified in the decleration and not in the implementation of the function.
#include <ArduinoOTA.h>

// ************ HASH DEFINES *******************
#define VERSION "2.1"
#define MAC_DELIMITER ":" // delimiter with which MAC address is separated
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
#define KEY_LEN  16 // lenght of PMK & LMK key (fixed at 16 for ESP)
#ifndef ESP_OK
  #define ESP_OK 0 // This is defined for ESP32 but not for ESP8266 , so define it
#endif
#define QUEUE_LENGTH 50 // max no of messages the ESP should queue up before replacing them, limited by amount of free memory
#define SENSOR_HEALTH_INTERVAL 30e3 // interval is millisecs to wait for sensor values before rasing a disconnect event
#define DEBOUNCE_INTERVAL 500 // button bounce interval in ms
#define DASHBOARD_UPDATE_INTERVAL 1e3 // interval is millisecs to publish updates to dashboard
#define SIGNAL_CHART_UPDATE_INTERVAL 1800e3 // no of seconds in which to update the signal stats
#define MAX_SIGNAL_ITEMS 30 // no of items the signal array can hold , I think the chart has issues if this is beyond 35
// ************ HASH DEFINES *******************

// ************ GLOBAL OBJECTS/VARIABLES *******************
const char* ssid = WiFi_SSID; // comes from config.h
const char* password = WiFi_SSID_PSWD; // comes from config.h
// need to include this file after ssid variable as I am using ssid inside espcontroller, not a good design but will sort this out later
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller
long last_sensor_check_time = 0; // last time when health check was published
long last_dashboard_refresh_time = 0; // last time when dashboard was refreshed with values
long last_signal_update_time = 0; // last time when signal stat was updated
bool sensor_connected = false; // flag to indicate if the controller is receiving messages from the sensor on a regular basis
bool motor_running = false; //flag to indicate if the motor is in running state or not
bool sump_empty = false; // Flag to store the status of sump
char tank_status[10] = "unknown"; // status of the Tank to be shown on the dashboard
short battery_percent = 0; // stores the battery % left for the sensor
long lastMillis = 0; // time when last button was pressed , for debouncing
extern "C"
{ 
  #include <lwip/icmp.h> // needed for icmp packet definitions , not sure what was this?
}
ArduinoQueue<espnow_message> structQueue(QUEUE_LENGTH);
ezLED  statusLED(STATUS_LED,CTRL_CATHODE);
ezLED  sumpLED(SUMP_LED,CTRL_ANODE);
ezLED  tankLED(TANK_LED,CTRL_ANODE);

static espnow_message emptyMessage;
espnow_message currentMessage = emptyMessage;
#if defined(ESP32)
esp_now_peer_info_t peerInfo; // This object must be a global object else the setting of peer will fail
#endif
#if USING(SECURITY)
//List of controllers(sensors) who will send messages to this receiver , only needed if security is in use
uint8_t controller_mac[][6] = CONTROLLERS; //from secrets.h
/* example entry : 
uint8_t controller_mac[2][6] = {  
   {0x4C, 0xF2, 0x32, 0xF0, 0x74, 0x2D} ,
   {0xC, 0xDD, 0xC2, 0x33, 0x11, 0x98}
}; */
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif

espnow_message myData;
espnow_device esp_ota_device;
Debounce sump_button(SUMP_PIN, 80, true); // SUMP_PIN is the pin, 80 is the delay in ms, true for INPUT_PULLUP.

// objects for ESP Dashboard
AsyncWebServer server(80);
/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&server); 
Card motor_card(&dashboard, STATUS_CARD, "Motor");
Card sump_card(&dashboard, STATUS_CARD, "Sump");
Card tank_card(&dashboard, STATUS_CARD, "Tank");
Card sensor_card(&dashboard, STATUS_CARD, "Sensor");
Card wifi_card(&dashboard, STATUS_CARD, "WiFi");
Card battery_card(&dashboard, PROGRESS_CARD, "Sensor Battery","%",0,100);
Chart signal_chart(&dashboard, BAR_CHART, "Signal History");
int signal_xaxis[MAX_SIGNAL_ITEMS]; // holds values of no of mins that have elapsed since startup
int signal_yaxis[MAX_SIGNAL_ITEMS]; // holds the value of signal at various intervals

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
 * Callback called on sending a message. It queues the incoming message into a queue
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
 * Prints initial info about the device and other important parameters
 */
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
 * configures wifi / OTA
 */
void configure_wifi()
{
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);// As this receiver will be receiving messages from the sensor , Wifi must be ON at all times
  //WiFi.mode(WIFI_AP_STA);
  WiFi.config(ESP_IP_ADDRESS, default_gateway, subnet_mask);//from secrets.h
  String device_name = DEVICE_NAME;
  device_name.replace("_","-");//hostname dont allow underscores or spaces
  WiFi.hostname(device_name.c_str());// Set Hostname.
  DPRINTLN("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) // has a timeout of 60 sec
  {
      DPRINTLN("Failed to connect to WiFi!");
      WiFi.setAutoReconnect(false);
      return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  /* Start AsyncWebServer */
  server.begin();
  ArduinoOTA.begin();
}

/*
 Turns the motor ON/OFF after checking the relavent conditions
 param: turn_on - bool , if true motore is turned on else off
 */
void set_motor_state(bool turn_on)
{
  if(turn_on)  
  {
    if(digitalRead(MOTOR_PIN))
      return; //motor is already ON, nothing to do
    // Turn ON motor only if sump is filled
    if(sump_button.read())
    {
      digitalWrite(MOTOR_PIN,HIGH);
      DPRINTLN("Turning motor ON");
      motor_running = true;
    }
    else
    {
      //DPRINTLN("Ignoring motor ON command as SUMP is empty");
    }
  }
  else
  {
    if(digitalRead(MOTOR_PIN)) // turn OFF motor only if it is ON
    {
      digitalWrite(MOTOR_PIN,LOW);
      DPRINTLN("Turning motor OFF");
      motor_running = false;
    }
  }
}


/*
 * Updates the dashboard cards and then calls dashboard.sendUpdates()
 */
void update_dashboard()
{
  if(motor_running)
    motor_card.update("running","success");
  else if(sump_empty)
    {motor_card.update("stopped","danger");}
  else
    {motor_card.update("stopped","idle");}

  if(sump_empty)
    sump_card.update("empty","danger");
  else
    {sump_card.update("filled","success");}

  if(sensor_connected)
    sensor_card.update("connected","success");
  else
    {sensor_card.update("disconnected","danger");}
  
  if(strcmp(tank_status,"filled") == 0)
    tank_card.update(tank_status,"success");
  else if(strcmp(tank_status,"empty") == 0)
    tank_card.update(tank_status,"danger");
  else
    tank_card.update(tank_status,"idle");

  if(WiFi.isConnected())
  {
    String str = "connected/Ch:" + String(getWiFiChannel());
    wifi_card.update(str,"success");
  }
  else
  {
    String str = "disconnected/Ch:" + String(getWiFiChannel());
    wifi_card.update(str,"idle");
  }


  battery_card.update(battery_percent);

  dashboard.sendUpdates();
}
/*
 * perform setup operations
 */
void setup() {
  {DBEGIN(115200);}
  printInitInfo();
  WiFi.mode(WIFI_AP_STA);

  // Set a custom MAC address for the device. This is helpful in cases where you want to replace the actual ESP device in future
  // A custom MAC address will allow all sensors to continue working with the new device and you will not be required to update code on all devices
  #ifdef DEVICE_MAC
  uint8_t customMACAddress[] = DEVICE_MAC; // defined in Config.h
  
  setCustomMAC(customMACAddress,false);
  #endif

  pinMode(STATUS_LED,OUTPUT);
  pinMode(SUMP_LED,OUTPUT);
  pinMode(TANK_LED,OUTPUT);
  pinMode(MOTOR_PIN,OUTPUT);
  statusLED.turnOFF();// turn status led off as it will be controlled by other conditions later in the program
  String device_name = DEVICE_NAME;
  device_name.replace("_","-");//hostname dont allow underscores or spaces
  WiFi.hostname(device_name.c_str());// Set Hostname.

  #if USING(WIFI_CONNECTION)
  configure_wifi();
  #endif

  // Set the device as a Station and Soft Access Point simultaneously
  // This has to be WIFI_AP_STA and not WIFI_STA, if set to WIFI_STA then it can only receive broadcast messages and stops receiving MAC specific messages.
  // The main problem seems to be caused by the station mode WiFi going into sleep mode while you have no work. This means that it does not listen to receive ESP-Now packets
  // and therefore they are lost. To solve this we will have to force our microcontroller to listen continuously, and this is achieved by turning it into an AP (Access Point)
  if(WiFi.isConnected()) // If WiFi is connected the channel is automatically set by the AP, so have to set the same for espnow
  {
    initilizeESP(MY_ROLE);
  }
  else // If we're not connected then choose the channel for espnow
  {
    initilizeESP(DEFAULT_CHANNEL,MY_ROLE,WiFi.getMode());
  }

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
}

/*
 * Proceses the message, in this case the message will either turn the motor ON/OFF or leave it as it is
 */
bool process_message(espnow_message msg)
{
  // status of the tank level is received in the intvalue1 field. 
  if(msg.intvalue1 == TANK_FULL_VALUE) // sensor indicates the tank is filled , turn OFF motor
  {
    set_motor_state(false);
    strcpy(tank_status,"filled");
    tankLED.turnOFF();
  }
  else // tank is empty , turn ON motor is sump is filled
  {
    set_motor_state(true);
    strcpy(tank_status,"empty");
    tankLED.turnON();
  }

  battery_percent = map_Generic(msg.floatvalue1,3.0,4.2,0,100);
  return true;
}


/*
 * monitors the conditions which affect the state of the motor
  if it is empty then raises an alarm by blinking LED
  If there is no connection to the sensor or the sump is empty , it turns off motor
 */
void monitor_motor_state()
{
  if(!sump_button.read())
  {
    sump_empty = true;
    // raise alarm , sump is empty
    if(sumpLED.getState() != LED_BLINKING)
    {
      set_motor_state(false);
      sumpLED.blink(500,500);
      DPRINTLN("Blinking sump LED...");
    }
  }
  else
  {
    sump_empty = false;
    if(sumpLED.getState() == LED_BLINKING)
    {
      sumpLED.cancel();
      DPRINTLN("Cancelled Blinking sump LED...");
    }
  }
  //monitor connection state and take action only if motor is ON
  if(!sensor_connected && digitalRead(MOTOR_PIN))
  {
    set_motor_state(false);
    DPRINTLN("Motor turned OFF due to lost connection to sensor");
  }
}

/*
 * updates the various stats for graphs , at present only signal graph. It pushes all values by one place to the left before updating a fresh value at the last
 */
void update_stats(bool force_update= false)
{
   if((millis() - last_signal_update_time > SIGNAL_CHART_UPDATE_INTERVAL) || force_update)
   {
    // shift all values by one place to the left
    byte i=0;
    for(i=0;i<MAX_SIGNAL_ITEMS-1;i++)
    {
      signal_xaxis[i] = signal_xaxis[i+1];
      signal_yaxis[i] = signal_yaxis[i+1];
    }
    // no update the last value
    signal_xaxis[i] = int(millis()/(60*1000));// value in minutes
    signal_yaxis[i] = sensor_connected?1:0; // I use a value 0 for signal absent and 1 for signal present
    //DPRINT(signal_xaxis[i]);DPRINT(","); DPRINTLN(signal_yaxis[i]);
    signal_chart.updateX(signal_xaxis,MAX_SIGNAL_ITEMS);
    signal_chart.updateY(signal_yaxis,MAX_SIGNAL_ITEMS);
    last_signal_update_time = millis();
   }
}

/*
 * monitors the status of the connection from the sensor, blinks LED if it loses connection
 */
void monitor_sensor_status()
{
  if(millis() - last_sensor_check_time > SENSOR_HEALTH_INTERVAL)
  {
    if(sensor_connected)
    {
      sensor_connected = false;// set the status to connected as process_message uses this flag
      update_stats(true);//whenever there is a change in sensor status , update the stats
    }
    strcpy(tank_status,"unknown");

    // Blink to warn that a message has not been received since SENSOR_HEALTH_INTERVAL
    if(statusLED.getState() != LED_BLINKING)
    {
      statusLED.blink(500,500);
      DPRINTLN("Connection to sensor lost...");
    }
  }
}

/*
 * runs the loop to check for incoming messages in the queue and monitors other things
 */
void loop() {
  if(!structQueue.isEmpty())
      currentMessage = structQueue.dequeue();
  
  if(currentMessage != emptyMessage)
  {
    //DPRINTFLN("Processing msg id:%lu,type:%d,int1:%d,fval1:%f",currentMessage.message_id,currentMessage.msg_type,currentMessage.intvalue1,currentMessage.floatvalue1);
    if(currentMessage.msg_type == ESPNOW_SENSOR)
    {
      if(!sensor_connected)
      {
        sensor_connected = true;// set the status to connected as process_message uses this flag
        update_stats(true);//whenever there is a change in sensor status , update the stats
      }
      process_message(currentMessage);
      DPRINTFLN("Processing msg id:%lu",currentMessage.message_id);
      last_sensor_check_time = millis(); // renew the last_sensor_check_time so that we know we're receiving messages regularly
      if(statusLED.getState() == LED_BLINKING)
      {
        statusLED.cancel(); //cancel blinking to indicate we're good
        DPRINTLN("Connection to sensor restored...");
      }
      statusLED.turnON();// ON indicates we're receiving messages regularly
      currentMessage = emptyMessage;
    }
  }
  monitor_motor_state();
  monitor_sensor_status();
  update_stats();

  //update the dashboard
  if(millis() - last_dashboard_refresh_time > DASHBOARD_UPDATE_INTERVAL)
  {
    update_dashboard();
    last_dashboard_refresh_time = millis();
  }

  statusLED.loop();
  sumpLED.loop();
  tankLED.loop();
  if(WiFi.isConnected())
    ArduinoOTA.handle();
}
