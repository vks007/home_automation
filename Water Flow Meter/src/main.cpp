
/*
 * Sketch for flow meter based on ESP. The ESP wakes up due to a pulse from the flow meter and sends the meter readings, waits for the flow to stop, waits for 
 * a certain time period and goes to sleep again. 

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
#define VERSION "1.0"
#define SENSOR_UPDATE_INTERVAL 1 //interval to update the sensor values
#define PUBLISH_INTERVAL 2 // interval in seconds at which the message is posted to MQTT when the ESP is awake , cannot be greater than IDLE_TIME
#define IDLE_TIME 15 //300 //idle time in sec beyond which the ESP goes to sleep , to be woken up only by a pulse from the meter
#define DEBOUNCE_INTERVAL 10 //debouncing time in ms for interrupts
#define PULSE_PER_LIT 292 //no of pulses the meter counts for 1 lit of water , adjust this for each water meter after calibiration
#define MAGIC_NUMBER 0x12345678
#define RTC_MEM_START 64
// ************ HASH DEFINES *******************

#include <Arduino.h>
#include "secrets.h"
#include "Config.h"
#include "Debugutils.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h" //include utility functions
#include <user_interface.h> // for RTC memory functions

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
ADC_MODE(ADC_VCC);//connects the internal ADC to VCC pin and enables measuring Vcc
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string

espnow_message myData;
volatile bool msgReceived = false; //flag to indicate if the ESP has received any message during its wake up cycle
#if USING(OTA) 
volatile bool ota_msg = false; // indicates if the esp has received a OTA message
volatile espnow_mode_t ota_mode = MODE_NORMAL; // mode in which the ESP starts, this is read from EEPROM in setup()
#endif
unsigned long start_time = millis(); // keeps track of the time ESP started, can be changed in between though
const unsigned short eeprom_start_add = sizeof(int); // starting address of EEPROm for use of this ESP. This is determined by the space espnowcontroller 
bool msgSent = false ; // indicates if the message is sent when ESP wakes up so that it doesnt send any other message till it kills power to itself

// takes to store its data which at present is only the WiFi channel number as integer, the rest till EEPROM_SIZE is available to this ESP to store its data
#if USING(SECURITY)
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h
#endif

volatile unsigned int pulses = 0; //stores the no of pulses detected by the sensor, it is reset after SENSOR_UPDATE_INTERVAL and the value stored in accumulated_pulses
volatile unsigned int accumulated_pulses = 0; //stores the no of pulses that haave been accumulated since the last SENSOR_UPDATE_INTERVAL
volatile unsigned int last_total_pulses = 0; //stores the  value of the last total pulses, used to detect if there is any change in the pulses
volatile unsigned int total_pulses = 0; //stores the total pulses since the last restart
volatile unsigned int pulse_rate = 0; //stores the pulse flow rate in pulses/min
volatile float flow_rate = 0.0;//flow rate is per minute
volatile float total_volume = 0.0;//stores the total volume of liquid since start
unsigned long last_publish_time = 0;//stores the no of ms since a message was last published , used to sleep the ESP beyond a certain value
unsigned long sleep_time_start = 0;
unsigned long sleep_time = 0;
unsigned long up_time = RTCmillis();
unsigned awake_count = 0;
struct RtcData {
  uint32_t magic_number;
  uint32_t awake_count;
};

os_timer_t publish_timer;
os_timer_t update_timer;
bool publish_tick = false;// flag to keep track of when to enable publishing of message
volatile unsigned long lastMicros;

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
  #if USING(OTA)
  if(!ota_msg && ota_mode!= MODE_OTA_START) 
  {
    msgReceived = true;
    if(msg.msg_type == ESPNOW_OTA)
      ota_msg = true;
    DPRINTF("Processing msg:%lu,%u,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.msg_type,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
  }
  else // ignore any messages if we are already in OTA mode
  #endif
    DPRINTF("Ignoring msg:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void printInitInfo()
{
  DPRINTFLN("Version:%s",compile_version);
  DPRINTFLN("Starting up as %s device",DEVICE_NAME);
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

    //Set the publish flag to false & reset time irrespective of if the msg publish is a success
    //If it fails , it will anyway retry after the next interval
    publish_tick = false;
    last_publish_time = millis();
    strcpy(myData.sender_mac,WiFi.macAddress().c_str()); //WiFi.softAPmacAddress()
    myData.intvalue1 = pulse_rate;
    myData.intvalue2 = total_pulses;
    myData.intvalue3 = awake_count;
    myData.intvalue4 = safeConvertUnsignedLongToInt((unsigned long)sleep_time/1000);
    myData.floatvalue1 = (float)flow_rate;
    myData.floatvalue2 = (float)total_volume;
    myData.chardata1[0] = '\0';
    strncpy(myData.chardata2,compile_version,15);//only copy the first 15 chars as compile_version is longer
    myData.chardata2[15] = '\0';//add the null character else it will result in overflow of memory

  }
  #if USING(OTA)
  else if(msg_type == ESPNOW_OTA)
  {
    myData.intvalue1 = ota_mode; // ota mode
    myData.intvalue2 = OTA_TIMEOUT; // ota timeout time in sec
    myData.floatvalue1 = OTA_TIMEOUT - (millis()- start_time)/1000; // time remaining for ota mode in secs
    strcpy(myData.chardata1,ESP_IP_ADDRESS.toString().c_str());
    strcpy(myData.chardata2,"");
  }
  #endif
  //generate a random value for the message id. It seems there is nothing I can do to genrate a random value as all random values need a seed
  // and that for a ESP is always constant. Hence I am trying to get a combination of the following 4 things, micros creates an almost true random number
  myData.message_id = WiFi.RSSI();// + micros();
    
  DPRINTFLN("Sending message Id:%u, Flow rate: %f, Volume: %f",myData.message_id,myData.floatvalue1,myData.floatvalue2);
  bool result = sendESPnowMessage(&myData,gatewayAddress,1,acknowledge);
  if (result == 0) {
    DPRINTLN("Delivered with success");}
  else {DPRINTFLN("Error sending/receipting the message, error code:%d",result);}
}


/*
* Processes incoming messages, for now its only OTA msg type, In future can code for more events
* For future events I will have to collect incoming messages in a queue and then process them
*/
void process_messages()
{
  #if USING(OTA)
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
  #endif
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

//system_get_rst_info ()
void publish_timer_callback(void *pArg) {
  
  if(last_total_pulses != total_pulses)
  {
    last_total_pulses = total_pulses;
    publish_tick = true;
  }
}


/*
  Timer callback function to update the various sensor values like flow/volume based on pulses
*/
void update_timer_callback(void *pArg) {
  // to make the following code atomic implement a critical section by disabling interrupts
  noInterrupts();
  accumulated_pulses = pulses;//collect the pulses into a temp variable as pulses can be updated simultaneously in interrupt
  pulses = 0;//reset pulses as this was needed to calculate rate per SENSOR_UPDATE_INTERVAL only
  interrupts();
  // end critical section

}

void IRAM_ATTR pulseHandler() {
  if((long)(micros() - lastMicros) >= DEBOUNCE_INTERVAL * 1000) {  //note DEBOUNCE_INTERVAL is in ms , so multiply by 1000 for microsec
    pulses += 1;
    lastMicros = micros();
  }
}

void timerInit(void) {
  os_timer_setfn(&publish_timer, publish_timer_callback, NULL);
  os_timer_arm(&publish_timer, 1000 * PUBLISH_INTERVAL, true);
  os_timer_setfn(&update_timer, update_timer_callback, NULL);
  os_timer_arm(&update_timer, 1000 * SENSOR_UPDATE_INTERVAL, true);
}

void light_sleep(){
    // for timer-based light sleep to work, the os timers need to be disconnected
    extern os_timer_t *timer_list;
    timer_list = nullptr; 
    WiFi.mode(WIFI_OFF);  // you must turn the modem off; using disconnect won't work
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    bool wakeup_pin_state = digitalRead(WAKEUP_PIN);
    gpio_pin_wakeup_enable(GPIO_ID_PIN(WAKEUP_PIN), wakeup_pin_state? GPIO_PIN_INTR_LOLEVEL:GPIO_PIN_INTR_HILEVEL);// only LOLEVEL or HILEVEL interrupts work, no edge, that's a CPU limitation
    DPRINTFLN("CPU going to sleep, pull WAKE_UP_PIN %s to wakeup",wakeup_pin_state?"LOW":"HIGH");DFLUSH();
    wifi_fpm_open();
    sleep_time_start = RTCmillis(); //record the time before sleeping
    wifi_fpm_do_sleep(0xFFFFFFF);  // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(10);  // it goes to sleep during this delay() and waits for an interrupt
    DPRINTLN(F("Woke up!"));  // the interrupt callback hits before this is executed*/
    timerInit(); //reinitialize the timers
    //sometimes the above statement gets printed before actually sleeping off, doesnt happen all the time though
    //But the code works as expected, the ESP sleeps after printing Woke up
 }

void setup() {
  DBEGIN(115200);
  printInitInfo();
  setCustomMAC(customMACAddress,true);
  
  // Write the awake count to RTC memory as 1 , this will be updated every time the ESP wakes up
  RtcData rtc_data;
  awake_count = 1;
  rtc_data.magic_number = MAGIC_NUMBER;
  rtc_data.awake_count = awake_count;
  system_rtc_mem_write(RTC_MEM_START, &rtc_data, sizeof(rtc_data));

  pinMode(WAKEUP_PIN, INPUT_PULLUP);//INPUT_PULLUP
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(PULSE_PIN, pulseHandler, RISING);
  timerInit();


  //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
  #if USING(EEPROM_STORE)
  EEPROM.begin(EEPROM_SIZE);
  #endif
  // Also used for storing OTA flag
  #if USING(OTA)
  ota_mode = EEPROM.get(eeprom_start_add,ota_mode);
  DPRINTFLN("Starting up in %s mode",ota_mode== MODE_OTA_START?"OTA":"ESPNOW");
  if(ota_mode == MODE_OTA_START)
  {
    setup_OTA();
    set_led(1);
    // TO DO : explore if there is a way to enable Serial Debug when in OTA mode even if it has been turned OFF in the Config.h
  }
  #endif
  // this will also execute if we get any junk value which is possible if we've never written to EEPROM ever
  DPRINTLN("Setup complete");
}

void loop() {
  #if USING(OTA)
  if(ota_mode== MODE_OTA_START) 
  {
    ArduinoOTA.handle();
    // countdown to the max time you should remain in the OTA mode before going back to sleep
    if((millis()- start_time) > OTA_TIMEOUT*1000)
    {
      set_led(0);
      DPRINTLN("No OTA file received, timing out of OTA mode");
      ota_mode = MODE_NORMAL; // This will make the code exit loop() by killing power to itself
      DFLUSH();
    }
  }
  else
  #endif
  {
    if(publish_tick)
    {
      send_message(ESPNOW_SENSOR);
    }
    unsigned long idle_time = millis() - last_publish_time;
    if(accumulated_pulses > 0)
    {
      // to make the following code atomic implement a critical section by disabling interrupts
      noInterrupts();
      unsigned long temp = accumulated_pulses;
      accumulated_pulses = 0;
      interrupts();
      // end critical section

      //I tried to get interval via micros() here but the ESP crashed and so I reverted to using SENSOR_UPDATE_INTERVAL here
      pulse_rate = (temp/(SENSOR_UPDATE_INTERVAL))*60.0; //pulse rate in pulses/min
      unsigned int pulse_per_lit = PULSE_PER_LIT;//assign a default

      // The flow meter i am using cannot measure low flow rates very accurately , so i adjust the pulse_per_litre value to 
      // compensate for the non linear values. I have obtained the below values by doing actual measurements of flow with a 
      // fixed amount of water
      if(pulse_rate <270)
        pulse_per_lit = 145;
      else if(pulse_rate <600)
        pulse_per_lit = 196;
      else if(pulse_rate <960)
        pulse_per_lit = 212;
      else if(pulse_rate <1200)
        pulse_per_lit = 224;
      else if(pulse_rate <2400)
        pulse_per_lit = 235;
      else if(pulse_rate <10000)
        pulse_per_lit = 260;
      else
        pulse_per_lit = 292;

      flow_rate = (float)pulse_rate/pulse_per_lit;
      // comparison to ULONG isnt working - not sure why
      // if((ULONG_MAX - total_pulses) <= temp) //ULONG = 4,29,49,67,295
        total_pulses += temp;
      // else
        // total_pulses = temp; //reset the pulses as we have reached the max limit of unsigned long data type

      float increment_volume = (float)temp/pulse_per_lit;
      total_volume += increment_volume;

    }
    
    if(idle_time >= IDLE_TIME * 1000)
    {
      DPRINT("going to sleep after being idle for :"); DPRINT(idle_time/1000);DPRINTLN(" sec");DFLUSH();
      light_sleep();
      //now that we're awake , add the sleep time to the previous value
      sleep_time += (RTCmillis() - sleep_time_start);
      awake_count++; //we're awake one more time , so increment the counter
      DPRINT("Awake count: "); DPRINTLN(awake_count);
      // Update the awake_count in RTC memory
      RtcData rtc_data;
      system_rtc_mem_read(RTC_MEM_START, &rtc_data, sizeof(rtc_data));
      rtc_data.awake_count = awake_count;
      system_rtc_mem_write(RTC_MEM_START, &rtc_data, sizeof(rtc_data));

      last_publish_time = millis();
      safedelay(10);
    }

    yield();

  }

}
