/*
 * This circuit and sketch is a water leakage sensor using a piezo, op amp and ESP. ESP transmits messages using espnow and not WiFi
 * The whole piezo sensor circuit gets it's power from the ESP GPIO pin. 
 * The flow is : ESP wakes up, powers the piezo sensor/op amp circuit, reads the ADC values , determines if a vibration is present or not based on its value.
 * If ADC value is above a certain threshold, it sends a message with ADC value as well as another value indicating vibration status. It then sleeps to conserve power.
 * If there is no vibration detected, it still sends a health check message and then goes to sleep to conserve power.
 * The whole circuit is mounted inside a flush tank which runs water when flushed and also runs water in an event of a leakage. The purpose
 * of this circuit is to detect the leakage and report that as an espnow message so that the corresponding automation in home assistant can take action.
 *
 * Detection logic (deliberately kept simple here, actual leak-vs-normal-flush decision is left to Home Assistant):
 * - ESP8266 deep sleep can only be woken by a timer, not by an external interrupt, so this is a polling design.
 * - When idle (no vibration seen), the ESP wakes every SLEEP_DURATION seconds, takes a burst of ADC samples, sends a
 *   heartbeat/idle message (intvalue1 = VIB_STATUS_IDLE) and goes back to deep sleep.
 * - As soon as vibration is detected, the ESP stays awake (no deep sleep) and keeps sampling. It sends an "ongoing"
 *   update (intvalue1 = VIB_STATUS_ONGOING) every VIBRATION_UPDATE_INTERVAL_MS with the elapsed duration, so Home
 *   Assistant can decide whether this is a normal ~30s flush or a longer leak condition.
 * - Once the signal has been below the noise threshold for VIBRATION_STOP_CONFIRM_COUNT consecutive checks, the
 *   event is considered over, a final message is sent (intvalue1 = VIB_STATUS_ENDED) with the total duration, and
 *   the ESP goes back to deep sleep.
 *
 * Message field usage (espnow_message):
 * - intvalue1  : vibration status - VIB_STATUS_IDLE(0) / VIB_STATUS_ONGOING(1) / VIB_STATUS_ENDED(2)
 * - intvalue2  : duration in seconds of the current/just-ended vibration event (0 for idle heartbeat)
 * - intvalue3  : millis() at time of sending, for debugging purposes
 * - floatvalue1: last sampled raw ADC vibration level (0-1023)
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
#define VERSION "1.0"
//Types of messages decoded via the signal pins
// ************ HASH DEFINES *******************

#include <Arduino.h>
#include "secrets.h"
#include "Config.h"
#include "Debugutils.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for struct of espnow message
#include "myutils.h" //include utility functions

#if USING(EEPROM_STORE)
  #define EEPROM_SIZE 64 // number of bytes to be allocated to EEPROM , for some reason even though I am using only 4+4 8 bytes, it reads back junk values so I increased it to 64
  // havent tried lower than 64
  #include <EEPROM.h> // to store WiFi channel number to EEPROM
#endif

// ************ GLOBAL OBJECTS/VARIABLES *******************
const char* ssid = WiFi_SSID; // comes from config.h
const char* password = WiFi_SSID_PSWD; // comes from config.h
// Note: ADC_MODE(ADC_VCC) is intentionally NOT used here, see note in Config.h - A0 is used to read the piezo/op-amp signal instead
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string

// vibration status values sent in myData.intvalue1
typedef enum {
  VIB_STATUS_IDLE    = 0, // no vibration detected, this is a periodic heartbeat message
  VIB_STATUS_ONGOING = 1, // vibration event in progress, sent periodically while it continues
  VIB_STATUS_ENDED   = 2  // vibration event has just ended, message carries the total duration
} vibration_status_t;

espnow_message myData;
volatile bool msgReceived = false; //flag to indicate if the ESP has received any message during its wake up cycle
unsigned long start_time = millis(); // keeps track of the time ESP started, can be changed in between though
const unsigned short eeprom_start_add = sizeof(int); // starting address of EEPROm for use of this ESP. This is determined by the space espnowcontroller 
const unsigned long sleep_duration = SLEEP_DURATION * 1e6; // deep sleep duration in microseconds

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
 * Callback called on receiving a message. This device does not act on incoming messages, it just logs them.
*/
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
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


/*
 * Takes a burst of ADC readings (ADC_SAMPLE_COUNT samples, ADC_SAMPLE_INTERVAL_MS apart) from the piezo/op-amp
 * circuit on A0 and returns the peak (max) raw value seen. A peak/max is used rather than a single reading or an
 * average because the piezo output is expected to be an AC-like signal, so a single sample could easily land on
 * a zero-crossing and miss a genuine vibration.
 */
int sampleVibrationLevel()
{
  int peak = 0;
  for(short i=0;i<ADC_SAMPLE_COUNT;i++)
  {
    int reading = analogRead(A0);
    if(reading > peak)
      peak = reading;
    delay(ADC_SAMPLE_INTERVAL_MS);
  }
  return peak;
}

/*
 * Builds and sends an espnow_message reporting the current vibration status.
 * status        : VIB_STATUS_IDLE / VIB_STATUS_ONGOING / VIB_STATUS_ENDED
 * duration_secs : elapsed (or total, for ENDED) duration of the vibration event, 0 for an idle heartbeat
 * adc_level     : last sampled raw ADC vibration level (0-1023)
 */
void send_message(vibration_status_t status, unsigned long duration_secs, int adc_level, bool acknowledge = true)
{
  myData.msg_type = ESPNOW_SENSOR;
  myData.intvalue1 = status;
  myData.intvalue2 = duration_secs;
  myData.floatvalue1 = adc_level;
  //generate a random value for the message id. It seems there is nothing I can do to generate a random value as all random values need a seed
  // and that for a ESP is always constant. Hence I am trying to get a combination of the following 4 things, micros creates an almost true random number
  myData.intvalue3 = 0;
  myData.intvalue4 = 0;
  myData.floatvalue2 = 0;
  myData.floatvalue3 = 0;
  myData.floatvalue4 = 0;
  //Set other values to send
  strcpy(myData.sender_mac,WiFi.macAddress().c_str()); //WiFi.softAPmacAddress()
  // If devicename is not given then generate one from MAC address stripping off the colon
  #ifndef DEVICE_NAME
    String wifiMacString = WiFi.macAddress();
    wifiMacString.replace(":","");
    snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
  #else
    strcpy(myData.device_name,DEVICE_NAME);
  #endif
  myData.chardata1[0] = '\0';
  snprintf(myData.chardata2, MAX_CHAR_DATA_LEN, "%s", compile_version);
  myData.message_id = millis();

  bool result = sendESPnowMessage(&myData,gatewayAddress,1,acknowledge);
  if (result == 0) {
    DPRINTLN("Delivered with success");}
  else {DPRINTFLN("Error sending/receipting the message, error code:%d",result);}

  
}


void setup() {
  DBEGIN(115200);
  DPRINTLN();
  printInitInfo();
  pinMode(SENSOR_POWER_PIN,OUTPUT);
  digitalWrite(SENSOR_POWER_PIN,SENSOR_POWER_LOGIC);//power up the piezo/op-amp sensor circuit
  delay(20);//give the sensor circuit a little time to settle before it's read
    
  setCustomMAC(customMACAddress,true);

  DPRINTLN("initializing espnow");
  #if USING(EEPROM_STORE)
    //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
    EEPROM.begin(EEPROM_SIZE);// size of the EEPROM to be allocated, 16 is the minimum
    initilizeESP(ssid,MY_ROLE,DEFAULT_CHANNEL);
  #else // this will not reply on a SSID and channel stored in EEPROM
    initilizeESP(DEFAULT_CHANNEL,MY_ROLE,WIFI_STA);
  #endif

  #if(USING(SECURITY))
    esp_now_set_kok(kok, 16);
  #endif

  // register callbacks for events when data is sent and data is received
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  #if(USING(SECURITY))
    refreshPeer(gatewayAddress, key,RECEIVER_ROLE);
  #else
    refreshPeer(gatewayAddress, NULL,RECEIVER_ROLE);
  #endif
  DPRINTLN("Setup complete");
}

/*
 * Puts the sensor circuit to sleep and deep-sleeps the ESP for SLEEP_DURATION seconds. Never returns (ESP reboots on wake).
 */
void goToSleep()
{
  DPRINTFLN("Going to sleep for %d secs",SLEEP_DURATION);
  digitalWrite(SENSOR_POWER_PIN,!SENSOR_POWER_LOGIC);//remove power to the sensor module to conserve battery while asleep
  DFLUSH();
  ESP.deepSleep(sleep_duration);
}

/*
 * Runs once per boot (the ESP reboots after every deep sleep). Takes an initial reading and either reports an idle
 * heartbeat and sleeps, or - if vibration is present - stays awake monitoring/reporting until the vibration ends.
 */
void loop() {
  #if USING(TEST_MODE)
  int adc_level = sampleVibrationLevel();
  vibration_status_t status = adc_level > ADC_NOISE_THRESHOLD ? VIB_STATUS_ONGOING : VIB_STATUS_IDLE;
  DPRINTFLN("Test mode, adc level:%d",adc_level);
  send_message(status,0,adc_level);
  delay(TEST_MESSAGE_INTERVAL_MS);
  #else
  int adc_level = sampleVibrationLevel();

  if(adc_level <= ADC_NOISE_THRESHOLD)
  {
    // nothing going on, just send a heartbeat so Home Assistant knows the device is alive, then sleep
    DPRINTFLN("Idle, adc level:%d",adc_level);
    send_message(VIB_STATUS_IDLE,0,adc_level);
    goToSleep();
  }

  // vibration detected - stay awake and keep monitoring/reporting until it stops
  unsigned long event_start = millis();
  unsigned long last_update = event_start;
  short idle_count = 0;
  DPRINTFLN("Vibration started, adc level:%d",adc_level);
  send_message(VIB_STATUS_ONGOING,0,adc_level);

  while(idle_count < VIBRATION_STOP_CONFIRM_COUNT)
  {
    delay(VIBRATION_POLL_INTERVAL_MS);
    adc_level = sampleVibrationLevel();
    if(adc_level <= ADC_NOISE_THRESHOLD)
      idle_count++;
    else
      idle_count = 0; // still vibrating, reset the stop-confirmation counter

    if(millis() - last_update >= VIBRATION_UPDATE_INTERVAL_MS)
    {
      unsigned long duration_secs = (millis() - event_start) / 1000;
      DPRINTFLN("Vibration ongoing, duration:%lus, adc level:%d",duration_secs,adc_level);
      send_message(VIB_STATUS_ONGOING,duration_secs,adc_level);
      last_update = millis();
    }
  }

  unsigned long total_duration_secs = (millis() - event_start) / 1000;
  DPRINTFLN("Vibration ended, total duration:%lus",total_duration_secs);
  send_message(VIB_STATUS_ENDED,total_duration_secs,adc_level);
  goToSleep();
  #endif
}
