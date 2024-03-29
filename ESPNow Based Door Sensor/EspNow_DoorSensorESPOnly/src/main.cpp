
/*
 * Sketch for a door sensor. The door sensor is based only on one ESP which deep sleeps and wakes up on a GPIO interrupt
 * It then reads the sensor value of the door contact and sends that to a ESP gateway via espnow
 * It then goes back to sleep
 * espnow takes ~ 2 sec to obtain the current channel of the SSID, so I store the same in the EEPROM memory and read it every time, saves a lot of time
 * As it rarely changes, the EEPROM isnt worn out. 
 * The entire sketch from start to finish takes less than 90ms to execute and go to sleep
 */

#define DEBUG (1) //can be either 0 or 1 , BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define

#include <Arduino.h>
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "secrets.h"
#include "espnowMessage.h" // for struct of espnow message
#include "Config.h"
#include "myutils.h" //include utility functions
#include "espnowUtil.h" //defines all utility functions for espnow functionality
#include <ESP_EEPROM.h> // to store espnow wifi channel no in eeprom for retrival later

#define VERSION "1.0"
//Types of messages decoded via the signal pins
#define SENSOR_NONE 0
#define SENSOR_OPEN 1
#define SENSOR_CLOSE 2
#define MSG_ON 1 //payload for ON
#define MSG_OFF 0//payload for OFF

short CURR_MSG = SENSOR_NONE;//This stores the message type deciphered from the states of the signal pins

ADC_MODE(ADC_VCC);//connects the internal ADC to VCC pin and enables measuring Vcc
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string

espnow_message myData;

/*
 * Callback when data is sent , It sets the bResultReady flag to true on successful delivery of message
 * The flag is set to false in the main loop where data is sent and then the code waits to see if it gets set to true, if not it retires to send
 */
esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) {
  bResult = (status == 0? true:false);
//  bResult = status;
  DPRINT("Last Packet Send Status:");
  DPRINTLN(status == 0 ? "Delivery Success" : "Delivery Fail");
  bResultReady = true;
});

/*
 * Callback called on receiving a message. Nothing to do here for now , I am not using this in the flow
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void light_sleep(){
    DPRINTLN("CPU going to sleep, pull WAKE_UP_PIN low to wake it");DFLUSH();
    WiFi.mode(WIFI_OFF);  // you must turn the modem off; using disconnect won't work
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    gpio_pin_wakeup_enable(SIGNAL_PIN, GPIO_PIN_INTR_LOLEVEL);// only LOLEVEL or HILEVEL interrupts work, no edge, that's a CPU limitation
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xFFFFFFF);  // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(10);  // it goes to sleep during this delay() and waits for an interrupt
    DPRINTLN(F("Woke up!"));  // the interrupt callback hits before this is executed*/
 }



void setup() {
  //Set the HOLD pin HIGH so that the ESP maintains power to itself. We will set it to low once we're done with the job, terminating power to ESP
  DBEGIN(115200);
  //Init Serial Monitor
  DPRINTLN("Starting up");
  //Initialize EEPROM , this is used to store the channel no for espnow in the memory, only stored when it changes which is rare
  EEPROM.begin(16);// 16 is the size of the EEPROM to be allocated, 16 is the minimum
  // For us to use Rx as input we have to define the pins as below else it would continue to be Serial pins
  if(SIGNAL_PIN == 3)
  {
    pinMode(SIGNAL_PIN, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type     
  }
  pinMode(SIGNAL_PIN, INPUT_PULLUP);
  
  
}


void loop() {
  // control will come here only when we wake up from light sleep , so have to start off with everything

  //Read the value of the sensor on the input pins asap
  if(digitalRead(SIGNAL_PIN) == HIGH)
    CURR_MSG = SENSOR_OPEN;
  else if(digitalRead(SIGNAL_PIN) == LOW)
    CURR_MSG = SENSOR_CLOSE;
  //else nothing to do, invalid mode
  DPRINTLN(digitalRead(SIGNAL_PIN));

  DPRINTLN("initializing espnow");
  Initilize_espnow();

  // register callbacks for events when data is sent and data is received
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  refreshPeer();

  // populate the values for the message
  strcpy(myData.device_name,DEVICE_NAME);
  myData.intvalue1 = (CURR_MSG == SENSOR_OPEN? MSG_ON:MSG_OFF);
  myData.intvalue2 = ESP.getVcc();
  myData.intvalue4 = 0;
  myData.floatvalue1 = 0;
  myData.floatvalue2 = 0;
  myData.floatvalue3 = 0;
  myData.floatvalue4 = 0;
  myData.chardata1[0] = '\0';
  strncpy(myData.chardata2,compile_version,15);//only copy the first 15 chars as compile_version is longer
  myData.chardata2[15] = '\0';//add the null character else it will result in overflow of memory
  //generate a random value for the message id. It seems there is nothing I can do to genrate a random value as all random values need a seed
  // and that for a ESP is always constant. Hence I am trying to get a combination of the following 4 things, micros creates an almost true random number
  myData.message_id = myData.intvalue1 + myData.intvalue2 + WiFi.RSSI() + micros();
  myData.intvalue3 = millis();// for debug purpuses, send the millis till this instant in intvalue3
  send_espnow_message(&myData);

  //sleep now
  light_sleep();

}
