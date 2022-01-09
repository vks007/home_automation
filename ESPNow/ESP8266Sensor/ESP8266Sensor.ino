/*
 * This sketch is a sample sketch for a ESP8266 sensor (controller) using ESP-now. It sends message to a registered slave.
 * TO DO :
 * - create an array of slaves and send to the first available one. Try others if one fails. 
 * - encryption isnt working. Even if I change the keys on the master, the slave is able to receieve the messages, so have to debug later
 * if you can solve the encryption issue, remove it as with excryption , an eSP8266 can only connect to 6 other peers, ESP32 can connect to 10 other
 * while without encryption they can connect to 20 peers
 * - put security code under #define to be able to turn it ON and OFF
 * - Change the role to COMBO for both slave and Controller so that Slave can also pass on administration messages to the controller.
 * DONE:
 * create unique message id, store last id in RTC memory - i decided agianst it as this might result in wear and tear of memory
 * use the device_name as a filed to set the device name rather than the mac address, default to mac if device name isnt provided.
 */
 
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "secrets.h"
#include "espnowMessage.h" // for struct of espnow message
#include "Config.h"

// REPLACE WITH RECEIVER MAC Address , This should be the address of the softAP (and NOT WiFi MAC addr obtained by WiFi.macAddress()) if the Receiver uses both, WiFi & ESPNow
// You can get the address via the command WiFi.softAPmacAddress() , usually it is one decimal no after WiFi MAC address
//uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0x96, 0x91, 0xB1};//A4:CF:12:96:91:B1 - This is the SoftAP MAC addr of the ESP32
uint8_t broadcastAddress[] = {0x62, 0x01, 0x94, 0x5C, 0xA1, 0x8D};//62:01:94:5C:A1:8D - This is the SoftAP of ESP01 module
//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //Use this if you want to broadcast to all slaves, you could also loop through and send to multiple slaves
//Note though that in broadcast mode the delivery of messages are auto marked as delivered.

#define SECURITY 1
uint8_t kok[16]= PMK_KEY_STR;//comes from secrets.h
uint8_t key[16] = LMK_KEY_STR;// comes from secrets.h

typedef struct esp_now_peer_info {
  u8 peer_addr[6];
  uint8_t channel;
  uint8_t encrypt;
}esp_now_peer_info_t;


unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings
unsigned int readingId = 0;
// Insert your SSID
constexpr char WIFI_SSID[] = primary_ssid;

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
  DPRINT("\r\nLast Packet Send Status:\t");
  DPRINTLN(status == 0 ? "Delivery Success" : "Delivery Fail");
  if(status != 0)
  {
    //WiFi.printDiag(Serial);
    if(wifi_get_channel() == 0)
      ESP.restart();
  }
});
 
void setup() {
  //Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

// Sometimes after loading the new firmware the channel doesnt change to the required one, I have to reset the board to do the same.
// It rather reports the channel as 0 , I will have to look into this and solve, I think I saw some discussion around this where ESP remembers the last channel used 
// I think I have to fo a WiFi.disconnect to circumvent this, will try this out.

  WiFi.disconnect(); // trying to see if the issue of sometimes it not setting the right channel gets solved by this.
  // To change the channel you have to first call wifi_promiscuous_enable(true) , change channel and then call wifi_promiscuous_enable(false)
  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  wifi_promiscuous_enable(true);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //Init ESP-NOW
  if (esp_now_init() != 0) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }
  // Set the PMK key
  esp_now_set_kok(kok, 16);
  
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
    
  //Add peer , There is no method in ESP8266 to add a peer by passing esp_now_peer_info_t object unlike ESP32
  if (esp_now_add_peer((u8*)broadcastAddress, ESP_NOW_ROLE_SLAVE, channel, kok, 16) != 0){
    DPRINTLN("Failed to add peer");
    return;
  }
  esp_now_set_peer_key((u8*)broadcastAddress, key, 16);

}
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    static espnow_message myData;
    // If devicename is not given then generate one from MAC address stripping off the colon
    if(DEVICE_NAME == "")
    {
      String wifiMacString = WiFi.macAddress();
      wifiMacString.replace(":","");
      snprintf(myData.device_name, 16, "%s", wifiMacString.c_str());
    }
    else
      strcpy(myData.device_name,DEVICE_NAME);
    myData.intvalue1 = random(1,1000);
    myData.intvalue2 = random(1,1000);
    myData.intvalue3 = random(1,1000);
    myData.intvalue4 = random(1,1000);
    myData.floatvalue1 = random(1,1000)/1.1;
    myData.floatvalue2 = random(1,1000)/1.1;
    myData.floatvalue3 = random(1,1000)/1.1;
    myData.floatvalue4 = random(1,1000)/1.1;
    strcpy(myData.chardata1,"wemos");
    strcpy(myData.chardata2,"esp01gw");

    
    if(myData.message_id <= UINT_MAX)
      myData.message_id++;
    else
      myData.message_id = 1; //reset if we've reached the limint of int
      
//    DPRINT("size of packet:");DPRINTLN(sizeof(myData));

    //Send message via ESP-NOW , the result here just indicates that a message was sent , not that it was receieved.
    // For that you have to see the status in OnDataSent()
    int result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == 0) {
      DPRINTLN("Sent with success");
    }
    else {
      DPRINTLN("Error sending the data");
    }
  }
}
