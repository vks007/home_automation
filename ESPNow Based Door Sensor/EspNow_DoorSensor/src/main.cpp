
/*
 * Sketch for a contact sensor. The contact sensor power is controlled by a ATTiny to take advantage of the ultra low sleep current of a ATTiny. 
 * This ESP when it receives power, holds it power by setting CH_HOLD pin to HIGH 
 * It then reads the sensor value of the contact contact and sends that to a ESP gateway via espnow
 * It then powers itself off by pulling down the CH_HOLD pin.
 * espnow takes ~ 2 sec to obtain the current channel of the SSID, so I store the same in the EEPROM memory and read it every time, saves a lot of time
 * As it rarely changes, the EEPROM isnt worn out. 
 * The entire sketch from start to finish takes less than 90ms to execute and power down
 * TO DO :
 * - have multiple slaves to which a message can be tranmitted in the order of preference
 * - implement OTA update via webpage, tried ElegantAsyncOTA (it crashes) and WebOTA (webpage doesnt open up) libraries but both didnt work. so will have to implement myself
 */

/*
// you can use the macros below to pass a string value in the build flags and use the same in the code.
//Currrently I am using a numeric value so its okay but for string you will have to wrap it in macro as below
#define ST(A) #A
#define STR(A) ST(A)

#ifdef DEVICE
#pragma message STR(DEVICE)
#endif
*/
// ************ HASH DEFINES *******************
#define MSG_WAIT_TIMEOUT 30 // time in ms to wait for receiving any incoming messages to this ESP , typically 10-40 ms
#define OTA_TIMEOUT 180 // time in seconds beyond which to come out of OTA mode
#define VERSION "2.4"
//Types of messages decoded via the signal pins
#define SENSOR_NONE 0
#define SENSOR_OPEN 1
#define SENSOR_CLOSE 2
#define MSG_ON 1 //payload for ON
#define MSG_OFF 0//payload for OFF
// ************ HASH DEFINES *******************

#include <Arduino.h>
#include "secrets.h"
#include "Config.h"
#include "Debugutils.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h" //include utility functions

#if USING(OTA) // EEPROM is needed when OTA is used so force define it , if not defined
  #include <ArduinoOTA.h> 
  #define EEPROM_STORE            IN_USE
#endif
#if USING(EEPROM_STORE)
  #define EEPROM_SIZE 64 // number of bytes to be allocated to EEPROM , for some reason even though I am using only 4+4 8 bytes, it reads back junk values so I increased it to 64
  // havent tried lower than 64
  #include <EEPROM.h> // to store WiFi channel number to EEPROM
#endif

// ************ GLOBAL OBJECTS/VARIABLES *******************
const char* ssid = WiFi_SSID; // comes from config.h
const char* password = WiFi_SSID_PSWD; // comes from config.h
short CURR_MSG = SENSOR_NONE;//This stores the message type deciphered from the states of the signal pins
ADC_MODE(ADC_VCC);//connects the internal ADC to VCC pin and enables measuring Vcc
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
espnow_message myData;
volatile bool msgReceived = false; //flag to indicate if the ESP has received any message during its wake up cycle
volatile bool ota_msg = false; // indicates if the esp has received a OTA message
volatile espnow_mode_t ota_mode = MODE_NORMAL; // mode in which the ESP starts, this is read from EEPROM in setup()
unsigned long start_time = millis(); // keeps track of the time ESP started, can be changed in between though
const unsigned short eeprom_start_add = sizeof(int); // starting address of EEPROm for use of this ESP. This is determined by the space espnowcontroller 
bool msgSent = false ; // indicates if the message is sent when ESP wakes up so that it doesnt send any other message till it kills power to itself
bool powered_down = false; // indicates if the ESP has been powered down after doing its job

// takes to store its data which at present is only the WiFi channel number as integer, the rest till EEPROM_SIZE is available to this ESP to store its data
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif
// ************ GLOBAL OBJECTS/VARIABLES *******************
// need to include this file after ssid variable as I am using ssid inside espcontroller, not a good design but will sort this out later
#include "espnowController.h" //defines all utility functions for sending espnow messages from a controller

/*
 * Callback when data is sent , It sets the bResultReady flag to true on successful delivery of message
 * The flag is set to false in the main loop where data is sent and then the code waits to see if it gets set to true, if not it retires to send
 */
esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) {
  deliverySuccess = status;
  DPRINT("OnDataSent:Last Packet delivery status:\t");
  DPRINTLN(status == 0 ? "Success" : "Fail");
  bResultReady = true;
});

/*
 * Callback called on receiving a message. if msg type is OTA then set relevant flags else just log the message
*/
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  if(!ota_msg && ota_mode!= MODE_OTA_START) 
  {
    msgReceived = true;
    if(msg.msg_type == ESPNOW_OTA)
      ota_msg = true;
    DPRINTF("Processing msg:%lu,%u,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.msg_type,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  }
  else // ignore any messages if we are already in OTA mode
    DPRINTF("Ignoring msg:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void printInitInfo()
{
  DPRINTFLN("Version:%s",compile_version);
  DPRINTLN("Starting up door sensor device");
  #if USING(SECURITY)
    DPRINTLN("Security ON");
  #else
    DPRINTLN("Security OFF");
  #endif
  String wifiMacString = WiFi.macAddress();
  DPRINTFLN("This device's MAC add: %s",wifiMacString.c_str());

}

#if USING(OTA)
void setup_OTA()
{
  WiFi.mode(WIFI_AP_STA);
  WiFi.config(ESP_IP_ADDRESS, default_gateway, subnet_mask);//from secrets.h
  String device_name = DEVICE_NAME;
  device_name.replace("_","-");//hostname dont allow underscores or spaces
  WiFi.hostname(device_name.c_str());// Set Hostname.
  WiFi.begin(ssid, password);
  DPRINTLN("Setting as a Wi-Fi Station..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DPRINT(".");
  }
  DPRINT("Station IP Address: ");
  DPRINTLN(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  
  ArduinoOTA.onStart([]() {
    start_time = millis();//reset the start time now that we've started OTA
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
  // Now that we've started in OTA mode, set the EEPROM flag to MODE_OTA_END so that when we start up in ESPNOW mode the next time, we know we are coming out of OTA mode
  // This is irrespective of whether OTA setup completes successfully or not
  // be aware dont set ota_mode to this future mode her, we're still in ota_mode = MODE_OTA_START
  espnow_mode_t mode  = MODE_OTA_END;
  EEPROM.write(eeprom_start_add,mode);
  EEPROM.commit();
  EEPROM.get(eeprom_start_add,mode);
  if(mode != MODE_OTA_END)
  {
    DPRINTFLN("OTA Flag write mismatch. written:%u , read back:%u",MODE_OTA_END,mode);
  }
  DPRINTLN("OTA set up successfully");

}
#endif

void send_message(msg_type_t msg_type, bool acknowledge = true)
{
  myData.msg_type = msg_type;
  if(msg_type == ESPNOW_SENSOR)
  {
    //Read the value of the sensor on the input pins asap , ATtiny can then remove the signal and the ESP wont care
    if(digitalRead(SIGNAL_PIN) == HIGH)
      CURR_MSG = SENSOR_OPEN;
    else if(digitalRead(SIGNAL_PIN) == LOW)
      CURR_MSG = SENSOR_CLOSE;
    //else nothing to do, invalid mode
    DPRINTLN(digitalRead(SIGNAL_PIN));

    DPRINTLN("initializing espnow");
    initilizeESP(ssid,MY_ROLE);

    // register callbacks for events when data is sent and data is received
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    #if USING(SECURITY)
      refreshPeer(gatewayAddress,key,RECEIVER_ROLE);
    #else
      refreshPeer(gatewayAddress,NULL,RECEIVER_ROLE);
    #endif

    // populate the values for the message
    // If devicename is not given then generate one from MAC address stripping off the colon
    #ifndef DEVICE_NAME
      String wifiMacString = WiFi.macAddress();
      wifiMacString.replace(":","");
      snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
    #else
      strcpy(myData.device_name,DEVICE_NAME);
    #endif
    strcpy(myData.sender_mac,WiFi.macAddress().c_str()); //WiFi.softAPmacAddress()
    myData.intvalue1 = (CURR_MSG == SENSOR_OPEN? MSG_ON:MSG_OFF);
    DPRINTLN(myData.intvalue1);
    myData.intvalue2 = ESP.getVcc();
    myData.floatvalue1 = 0;
    myData.chardata1[0] = '\0';
    strncpy(myData.chardata2,compile_version,15);//only copy the first 15 chars as compile_version is longer
    myData.chardata2[15] = '\0';//add the null character else it will result in overflow of memory

  }
  else if(msg_type == ESPNOW_OTA)
  {
    myData.intvalue1 = ota_mode; // ota mode
    myData.intvalue2 = OTA_TIMEOUT; // ota timeout time in sec
    myData.floatvalue1 = OTA_TIMEOUT - (millis()- start_time)/1000; // time remaining for ota mode in secs
    strcpy(myData.chardata1,ESP_IP_ADDRESS.toString().c_str());
    strcpy(myData.chardata2,"");
  }
  //generate a random value for the message id. It seems there is nothing I can do to genrate a random value as all random values need a seed
  // and that for a ESP is always constant. Hence I am trying to get a combination of the following 4 things, micros creates an almost true random number
  myData.message_id = WiFi.RSSI();// + micros();
  myData.intvalue3 = millis();// for debug purpuses, send the millis till this instant in intvalue3
  myData.intvalue4 = 0;
  myData.floatvalue2 = 0;
  myData.floatvalue3 = 0;
  myData.floatvalue4 = 0;
    
  bool result = sendESPnowMessage(&myData,gatewayAddress,1,acknowledge);
  if (result == 0) {
    DPRINTLN("Delivered with success");}
  else {DPRINTFLN("Error sending/receipting the message, error code:%d",result);}
}

/*
* Turns LED off after a predermined total time, kills time via delay() if ESP hasnt been ON for that certain time
* state = true - ON , state = false - OFF
*/
void set_led(bool state)
{
  #if(USING(STATUS_LED))
    digitalWrite(LED_GPIO,LED_INVERTED?state:!state); // turn OFF the LED
  #endif
}

void toggle_led()
{
  #if(USING(STATUS_LED))
    digitalWrite(LED_GPIO,!(digitalRead(LED_GPIO))); // turn OFF the LED
  #endif
}

/*
* Processes incoming messages, for now its only OTA msg type, In future can code for more events
* For future events I will have to collect incoming messages in a queue and then process them
*/
void process_messages()
{
  if(ota_msg)
  {
    // write MODE_OTA_START in EEPROM and restart the ESP
    ota_mode  = MODE_OTA_START;
    EEPROM.write(eeprom_start_add,ota_mode);
    EEPROM.commit();
    //DPRINTFLN("OTA Flag read back from EEPROM %u",EEPROM.get(eeprom_start_add,ota_mode));
    send_message(ESPNOW_OTA,false);
    DPRINTLN("msg sent to Gateway to confirming receipt of OTA message. Going to restart the ESP for OTA mode...");
    DFLUSH();
    ESP.restart();
  }
  msgReceived = false; //now that we've processed the message , clear the flag
  // You can code for future events here
}

/*
* scan for received messages
*/
void scan_for_messages()
{
  // Wait for some time to see if we haev any service message for this ESP
  for(byte i=0;i<MSG_WAIT_TIMEOUT;i++)
  {
    delay(1);
    if(msgReceived)
    {
      process_messages();
      break;
    }
    yield();
  }
}

void setup() {
  //Set the HOLD pin HIGH so that the ESP maintains power to itself. We will set it to low once we're done with the job, terminating power to ESP
  if(HOLD_PIN == 1 || HOLD_PIN == 3)
  {
    pinMode(HOLD_PIN, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type
  }
    pinMode(HOLD_PIN, OUTPUT);
  if(HOLDING_LOGIC == LOGIC_NORMAL)
    digitalWrite(HOLD_PIN, HIGH);  // sets HOLD_PIN to high
  else // LOGIC_INVERTED
    digitalWrite(HOLD_PIN, LOW);  // sets HOLD_PIN to high
  DBEGIN(115200);
  
  #if(USING(STATUS_LED))
    pinMode(LED_GPIO, OUTPUT);
  #endif  

  printInitInfo();
  setCustomMAC(customMACAddress,true);


  //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
  // Also used for storing OTA flag
  EEPROM.begin(EEPROM_SIZE);
  ota_mode = EEPROM.get(eeprom_start_add,ota_mode);
  DPRINTFLN("Starting up in %s mode",ota_mode== MODE_OTA_START?"OTA":"ESPNOW");
  if(ota_mode == MODE_OTA_START)
  {
    #if USING(OTA)
      setup_OTA();
      set_led(true);
      // TO DO : explore if there is a way to enable Serial Debug when in OTA mode even if it has been turned OFF in the Config.h
    #endif
  }
  else // this will also execute if we get any junk value which is possible if we've never written to EEPROM ever
  {
    // For us to use Rx as input we have to define the pins as below else it would continue to be Serial pins
    if(SIGNAL_PIN == 1)
    {
      pinMode(SIGNAL_PIN, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type     
    }
    pinMode(SIGNAL_PIN, INPUT_PULLUP);
    
    //Init Serial Monitor
    DPRINTLN("Starting up");

    // The following code is to wait for some time before reading the value of the SIGNAL_PIN for a bouncy contact sensor
    // The one which bounces a few times before settling in on the final value
    #ifdef BOUNCE_DELAY
      safedelay(BOUNCE_DELAY*1000);
    #endif


  }
}


void loop() {
  if(ota_mode== MODE_OTA_START) 
  {
    #if USING(OTA)
      ArduinoOTA.handle();
      // countdown to the max time you should remain in the OTA mode before going back to sleep
      if((millis()- start_time) > OTA_TIMEOUT*1000)
      {
        set_led(false);
        DPRINTLN("No OTA file received, timing out of OTA mode");
        ota_mode = MODE_NORMAL; // This will make the code exit loop() by killing power to itself
        DFLUSH();
      }
    #endif
  }
  else
  {
    if(!msgSent)
    {
      //send_message(ESPNOW_SENSOR,true);
      toggle_led();
      #if !USING(TEST_MODE) // If not testing mode only then set the flag as below
        msgSent = true; //once a message is sent, set it to true, this will prevent any other message being sent as it takes some finite time to kill power to the ESP
                        // hence loop() keeps executing and sending messages
      #endif // else dont set the flag and the loop will keep sending the message
    }
    // I could keep the scan_for_messages() in the if block above so it executes only once but no harm in executing it repeatedly so letting it there
    // scan for messages sent to this ESP via espnow , this could be any message, one being OTA type message
    scan_for_messages();
    if(!msgReceived) //no messages are received to process, turn off led and go to sleep
    {
      #if USING(TEST_MODE)
      if(millis() >= 5000)
      {
        // Now you can kill power
        DPRINTLN("powering down");
        powered_down = true;
        DFLUSH();
        if(HOLDING_LOGIC == LOGIC_NORMAL)
          digitalWrite(HOLD_PIN, LOW);  // cut power to the ESP
        else // LOGIC_INVERTED
          digitalWrite(HOLD_PIN, HIGH);  // cut power to the ESP
      }
      delay(500); // we're in test mode, kill time before looping again
      #else
      if(!powered_down)
      {
        set_led(false);
        // Now you can kill power
        DPRINTLN("powering down");
        powered_down = true;
        DFLUSH();
        // if(HOLDING_LOGIC == LOGIC_NORMAL)
        //   digitalWrite(HOLD_PIN, LOW);  // cut power to the ESP
        // else // LOGIC_INVERTED
        //   digitalWrite(HOLD_PIN, HIGH);  // cut power to the ESP
      } // else nothing to do, leep looping if ESP is powered
      #endif
    }
  }

}
