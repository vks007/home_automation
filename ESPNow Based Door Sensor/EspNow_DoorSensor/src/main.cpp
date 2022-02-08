
/*
 * Sketch for a door sensor. The door sensor power is controlled by a ATTiny to take advantage of the ultra low sleep current of a ATTiny. 
 * This ESP when it receives power, holds it power by setting CH_HOLD pin to HIGH 
 * It then reads the sensor value of the door contact and sends that to a ESP gateway via espnow
 * It then powers itself off by pulling down the CH_HOLD pin. 
 * If TESTING_MODE is ON then we can use this ESP without the ATTINY to test the setup. In this mode it does not power itself OFF and keeps toggling the sensor state
 * In testing mode, it keeps toggling the door sensor contact ON and OFF. It also repurposes the Rx & Tx pin for Serial instead of GPIO pins for ESP01
 * TO DO :
 * - store the last used channel in the RTC memory , it currently spends ~2 sec in obtaining the channel
 * - have multiple slaves to which a message can be tranmitted in the order of preference
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
//#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "secrets.h"
#include "espnowMessage.h" // for struct of espnow message
#include "Config.h"
#include <time.h>
#include "myutils.h" //include utility functions like read/write RTC mem
#include "espnowUtil.h" //defines all util functions for espnow functionality

// This is how you assign a numeric value to a #define constant
#define TESTING_MODE (0) //can be either 0 or 1

#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#define SLEEP_TIME 60e6 // sleep time interval in microseconds

#define VERSION "2.2"
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
//Types of messages decoded via the signal pins
#define SENSOR_NONE 0
#define SENSOR_WAKEUP 1
#define SENSOR_OPEN 2
#define SENSOR_CLOSED 3
#define SENSOR_IDLE 4

#define MSG_ON 1 //payload for ON
#define MSG_OFF 0//payload for OFF
#if (TESTING_MODE)
  short CURR_MSG = SENSOR_OPEN;//This stores the message type deciphered from the states of the signal pins
#else
  short CURR_MSG = SENSOR_NONE;//This stores the message type deciphered from the states of the signal pins
#endif

ADC_MODE(ADC_VCC);//connects the internal ADC to VCC pin and enables measuring Vcc

espnow_message myData;

// MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
// You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address

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
 * Callback called on receiving a message. Nothing to do here for now
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void setup() {
  //Set the HOLD pin HIGH so that the ESP maintains power to itself. We will set it to low once we're done with the job, terminating power to ESP
  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, HIGH);  // sets GPIO0 to high (this holds CH_PD high even if the input signal goes LOW)
  // For us to use Rx and Tx pin as inputs we have to define the pins as below else they would continue to be Serial pins
  #if (!TESTING_MODE)
  if(SIGNAL_PIN0 == 1 || SIGNAL_PIN0 == 3)
  {
    pinMode(SIGNAL_PIN0, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type
  }
  if(SIGNAL_PIN1 == 1 || SIGNAL_PIN1 == 3)
  {
    pinMode(SIGNAL_PIN1, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type     
  }
  pinMode(SIGNAL_PIN0, INPUT_PULLUP);
  pinMode(SIGNAL_PIN1, INPUT_PULLUP);
  #else
    delay(1000); //introducing a delay in testing mode with DEBUG ON as the serial monitor on platformio isn't fast enough to catch messages
  #endif
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN("Starting up");

  Initilize_espnow();

  // register callbacks for events when data is sent and data is received
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  refreshPeer();

  // char device_id[13];
  // String wifiMacString = WiFi.macAddress();
  // wifiMacString.replace(":","");
  // snprintf(device_id, 13, "%s", wifiMacString.c_str());
  // strcpy(device_id,wifiMacString.c_str());
  // DPRINTF("deviceid:%s\n",device_id);

 
  // The main code below loops N times if we're testing to give 10 readings, else it reads once and then kills power to itself
  #if (TESTING_MODE)
    size_t loop_count = 3;
  #else
    size_t loop_count = 1;
  #endif

  for (size_t i = 0; i < loop_count; i++)
  {
    //TO DO : you can read the input values in a single statement directly from registers and then compare using a mask
    // TO DO : Shift the reading of pins to before reading config so that even if ATTiny removes the signal, ESP can still take its own time in publishing the message
    //Read the type of message we've got from the ATiny
    #if (TESTING_MODE)
      // Keep toggling the message type in testing mode
      CURR_MSG = (CURR_MSG== SENSOR_OPEN?SENSOR_CLOSED:SENSOR_OPEN);
    #else
    if((digitalRead(SIGNAL_PIN0) == LOW) && (digitalRead(SIGNAL_PIN1) == LOW))
      CURR_MSG = SENSOR_WAKEUP;
    else if((digitalRead(SIGNAL_PIN0) == LOW) && (digitalRead(SIGNAL_PIN1) == HIGH))
      CURR_MSG = SENSOR_OPEN;
    else if((digitalRead(SIGNAL_PIN0) == HIGH) && (digitalRead(SIGNAL_PIN1) == LOW))
      CURR_MSG = SENSOR_CLOSED;
    //else nothing to do, invalid mode
    #endif

    // populate the values for the message
    strcpy(myData.device_name,DEVICE_NAME);
    myData.intvalue1 = (CURR_MSG == SENSOR_OPEN? MSG_ON:MSG_OFF);
    myData.intvalue2 = ESP.getVcc();
    myData.intvalue4 = CURR_MSG;
    myData.floatvalue1 = digitalRead(SIGNAL_PIN0);
    myData.floatvalue2 = digitalRead(SIGNAL_PIN1);
    myData.floatvalue3 = 0;
    myData.floatvalue4 = 0;
    myData.chardata1[0] = '\0';
    strncpy(myData.chardata2,compile_version,15);//only copy the first 15 chars as compile_version is longer
    myData.chardata2[15] = '\0';//add the null character else it will result in overflow of memory
    //generate a random value for the message id
    srand(time(0));
    myData.message_id = rand();
    myData.intvalue3 = millis();// for debug purpuses, send the millis till this instant in intvalue3
    send_espnow_message(&myData);

    #if (TESTING_MODE)// dont kill power to itself in testing mode, delay and keep continuing
    delay(5000);
    #endif
  }
    // Now you can kill power
    DPRINTLN("powering down");
    digitalWrite(HOLD_PIN, LOW);  // set GPIO 0 low this takes CH_PD & powers down the ESP

}


void loop() {
//nothing to do here, it takes a few ms for the ESP to power down so the control will come here during that time
}
