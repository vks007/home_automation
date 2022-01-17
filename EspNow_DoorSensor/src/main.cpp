
/*
 * Sketch for a door sensor. The door sensor power is controlled by a ATTiny to take advantage of the ultra low sleep current of a ATTiny. 
 * This ESP when it receives power, holds it power by setting CH_HOLD pin to HIGH 
 * It then reads the sensor value of the door contact and sends that to a ESP gateway via espnow
 * It then powers itself off by pulling down the CH_HOLD pin. 
 * If TESTING_MODE is defined then we can use this ESP without the ATTINY to test the setup. In this mode it does not power itself OFF. 
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

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "secrets.h"
#include "espnowMessage.h" // for struct of espnow message
#include "Config.h"
#include<time.h>

// This is how you assign a numeric value to a #define constant
#define TESTING_MODE (0)

//#define DEBUG //BEAWARE that this statement should be before #include "Debugutils.h" else the macros wont work as they are based on this #define
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#define WAIT_TIMEOUT 50 // time in millis to wait for acknowledgement of the message sent
#define MAX_MESSAGE_RETRIES 4 // No of times a message is retries to be sent before dropping the message
#define SLEEP_TIME 60e6 // sleep time interval in microseconds

#define VERSION "2.2"
const char compile_version[] = VERSION " " __DATE__ " " __TIME__; //note, the 3 strings adjacent to each other become pasted together as one long string
//Types of messages decoded via the signal pins
#define SENSOR_NONE 0
#define SENSOR_WAKEUP 1
#define SENSOR_OPEN 2
#define SENSOR_CLOSED 3
#define MAX_MQTT_CONNECT_RETRY 4 //max no of retries to connect to MQTT server

#define MSG_ON 1 //payload for ON
#define MSG_OFF 0//payload for OFF
#if (TESTING_MODE)
  short CURR_MSG = SENSOR_OPEN;//This stores the message type deciphered from the states of the signal pins
#else
  short CURR_MSG = SENSOR_NONE;//This stores the message type deciphered from the states of the signal pins
#endif

ADC_MODE(ADC_VCC);//connects the internal ADC to VCC pin and enables measuring Vcc


#define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver

typedef struct esp_now_peer_info {// this is defined in esp_now.h for ESP32 but not in espnow.h for ESP8266
  u8 peer_addr[6];
  uint8_t channel;
  uint8_t encrypt;
}esp_now_peer_info_t;

espnow_message myData;
constexpr char WIFI_SSID[] = primary_ssid;// from secrets.h
volatile bool bSuccess = false;
volatile bool bResultReady = false;

// MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
// You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address

/*
 * Gets the WiFi channel of the SSID of your router, It has to be on the same channel as this one as the receiver will listen to ESPNow messages on the same 
 * While theritically it should be possible for WiFi and ESPNow to work on different channels but it seems due to some bug (or behavior) this isnt possible with espressif
 */
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// callback when data is sent
esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) {
  bSuccess = (status == 0 ? true : false);
  DPRINT("\r\nLast Packet Send Status:\t");
  DPRINTLN(status == 0 ? "Delivery Success" : "Delivery Fail");
  bResultReady = true;
});

/*
 * Callback called on sending a message.
 */
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  espnow_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  DPRINTF("OnDataRecv:%lu,%d,%d,%d,%d,%f,%f,%f,%f,%s,%s\n",msg.message_id,msg.intvalue1,msg.intvalue2,msg.intvalue3,msg.intvalue4,msg.floatvalue1,msg.floatvalue2,msg.floatvalue3,msg.floatvalue4,msg.chardata1,msg.chardata2);
};

void setup() {
  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, HIGH);  // sets GPIO0 to high (this holds CH_PD high even if the input signal goes LOW)
  // For us to use Rx and Tx pin as inputs we have to define the pins as below else they would continue to be Serial pins
  #if (!TESTING_MODE)
  if(SIGNAL_PIN0 == 1 || SIGNAL_PIN0 == 3)
  {
    pinMode(SIGNAL_PIN0, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type
    pinMode(SIGNAL_PIN1, FUNCTION_3);//Because we're using Rx & Tx as inputs here, we have to set the input type     
  }
  pinMode(SIGNAL_PIN0, INPUT_PULLUP);
  pinMode(SIGNAL_PIN1, INPUT_PULLUP);
  #endif
  //Init Serial Monitor
  DBEGIN(115200);
  DPRINTLN("Starting up");
  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);
// Sometimes after loading the new firmware the channel doesnt change to the required one, I have to reset the board to do the same.
// It rather reports the channel as 0 , I will have to look into this and solve, I think I saw some discussion around this where ESP remembers the last channel used 
// I think I have to fo a WiFi.disconnect to circumvent this, will try this out.

  WiFi.disconnect(); // trying to see if the issue of sometimes it not setting the right channel gets solved by this.
  // To change the channel you have to first call wifi_promiscuous_enable(true) , change channel and then call wifi_promiscuous_enable(false)
//  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  wifi_promiscuous_enable(true);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(false);
  delay(10);
  short ch = wifi_get_channel();
  DPRINTFLN("channel:%d",ch);
  //strange behavior : If I define ch as byte and make the comparison below , the ESP resets due to WDT and then hangs
  if(ch == 0)
  {
    DPRINTLN("WiFi Channel not set properly, restarting");
    ESP.restart();
  }
//  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //Init ESP-NOW
  if (esp_now_init() != 0) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(MY_ROLE);
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  //Add peer , There is no method in ESP8266 to add a peer by passing esp_now_peer_info_t object unlike ESP32
  if (esp_now_add_peer((u8*)gatewayAddress, RECEIVER_ROLE, channel, NULL, 0) != 0){
    DPRINTLN("Failed to add peer");
    return;
  }

  // char device_id[13];
  // String wifiMacString = WiFi.macAddress();
  // wifiMacString.replace(":","");
  // snprintf(device_id, 13, "%s", wifiMacString.c_str());
  // strcpy(device_id,wifiMacString.c_str());
  // DPRINTF("deviceid:%s\n",device_id);

 
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

  // populate the fixed values for a message
  strcpy(myData.device_name,DEVICE_NAME);
  myData.intvalue1 = (CURR_MSG == SENSOR_OPEN? MSG_ON:MSG_OFF);
  myData.intvalue2 = ESP.getVcc();
  myData.intvalue3 = digitalRead(SIGNAL_PIN0);
  myData.intvalue4 = digitalRead(SIGNAL_PIN1);
  myData.floatvalue1 = 0;
  myData.floatvalue2 = 0;
  myData.floatvalue3 = 0;
  myData.floatvalue4 = 0;
  myData.chardata1[0] = '\0';
  strncpy(myData.chardata2,compile_version,15);//only copy the first 15 chars as compile_version is longer
  myData.chardata2[15] = '\0';//add the null character else it will result in overflow of memory
  //generate a random value for the message id
  srand(time(0));
  myData.message_id = rand();
  // try to send the message MAX_MESSAGE_RETRIES times if it fails
  for(short i = 0;i<MAX_MESSAGE_RETRIES;i++)
  {
    int result = esp_now_send(gatewayAddress, (uint8_t *) &myData, sizeof(myData));
    // If devicename is not given then generate one from MAC address stripping off the colon
    long waitTimeStart = millis();
    if (result == 0) DPRINTLN("Sent with success");
    else DPRINTLN("Error sending the data");
    //get a confirmation of successful delivery , else will try again
    while(!bResultReady && ((millis() - waitTimeStart) < WAIT_TIMEOUT))
    {
      delay(1);
    }
    bResultReady = false; // prepare for next iteration
    if(bSuccess)
      break;
  }

  #if (TESTING_MODE)
  delay(5000);
  #else
  //Power down the ESP by writing LOW on HOLD PIN
  //DPRINTLN("powering down");
  digitalWrite(HOLD_PIN, LOW);  // set GPIO 0 low this takes CH_PD & powers down the ESP
  #endif

}


void loop() {
//nothing to do here, it takes a few ms for the ESP to power down so the control will come here during that time
}
