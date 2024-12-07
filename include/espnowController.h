/*
Utility file which encapsulates the functionality of initializing espnow on a controller (aka sensor). It has the following features
* - Single code base works with both ESP8266 and ESP32
* - Scans the WiFi channel number for the peer using provided SSID and stores it in EEPROM for later use. 
*  This saves ~2 sec time. The ESP controller is required to be on the same channel as the Slave else messages wont be received
* - Retries the sending of a failed message a certain no of times (passed as parameter)
* - Retries refreshing of WiFi channel no, if sending fails only once, if its required to scan the channel again, set channelRefreshed=false
*   from the calling code
* - Initilizes the espnow for espnow functions , sets role etc
* - refreshes the peer. This deletes an existing peer and adds it. This is to be called initially to add a peer
*   but can be called later too when the channel no changes in between and so the peer needs to be refreshed
  - Monitors the delivery success of the message subject to a timeout - WAIT_TIMEOUT
* Usage Steps :
  - Define the following variables in the calling code : 
    bool deliverySuccess , bool bResultReady , #define WIFI_SSID
  - Call EEPROM.begin(size) , to initialize the EEPROM. IT is used to store the channel no for future use
  - call initilizeESP - to initialize the ESP , aps in the SSID the Slave connects to and the role in case of ESP8266
    For ESP32 role is ignored
  - register the OnDatasent & onDatareceive callbacks in the calling code
  - Call refreshPeer() - passing in ther gateway address of Slave and ROLE of Slave. This concludes the setup process
  - Call sendESPnowMessage() to send a message of type espnow_message 
    monitor the delivery success of the message send via the flag deliverySuccess until bResultReady is not set

TO DO list:
  - refreshPeer() function when called from within sendESPnowMessage needs access to a variable WIFI_SSID in order to rescan the 
    SSID channel number, I have to remove this and paramterize this later
  - Security is still not tested fully although code for that is present


*/
#pragma once
#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <espnow.h>
#elif defined(ESP32)
#include <WiFi.h> // for WiFi functions, this needs to be added as a library dependency in platformio , lib name : WiFi
#include <esp_wifi.h> 
#include <esp_now.h>
#endif
#include "espnowMessage.h" // for struct of espnow message
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#if USING(EEPROM_STORE)
#include <EEPROM.h>
#endif

// ************ GLOBAL OBJECTS/VARIABLES *******************

bool channelRefreshed = false;//tracks the status of the change in wifi channel , true -> wifi channel has been refreshed
volatile uint8_t deliverySuccess = 9; //0 means success , non zero are error codes
volatile bool bResultReady = false;
// it is set to false every time the ESP boots
// ************ GLOBAL OBJECTS/VARIABLES *******************


// ************ HASH DEFINES *******************
#define KEY_LEN  16 // lenght of PMK & LMK key (fixed at 16 for ESP)
#define WAIT_TIMEOUT 25 // time in millis to wait for acknowledgement of the message sent
#define CONNECTION_RETRY_INTERVAL 30 // time is secs to wait before refreshing the connection in case of failure
#define DEFAULT_ESPNOW_CHANNEL         1 // default channel for espnow in case retrieving of channel from SSID fails
#define MAX_SSID 50
//Define the esp_now_peer_info if we're working with esp8266, for ESP32 its already defined
#if defined(ESP8266)
typedef struct esp_now_peer_info {// this is defined in esp_now.h for ESP32 but not in espnow.h for ESP8266
  u8 peer_addr[6];
  uint8_t channel;
  uint8_t encrypt;
}esp_now_peer_info_t;
#endif
// ESP32 does not use the concept of roles, it is only required for ESP8266. It is already defined for ESP8266
// But then to keep the code consistent and avoid a lot of #ifdef I am defining this enum for ESP32
#if defined(ESP32)
enum	esp_now_role	{
	 ESP_NOW_ROLE_IDLE	=	0, // no interface
	 ESP_NOW_ROLE_CONTROLLER, //priority is given to the station interface
	 ESP_NOW_ROLE_SLAVE, //priority is given to the SoftAP interface
	 ESP_NOW_ROLE_COMBO,	 //	priority is given to the station interface
	 ESP_NOW_ROLE_MAX, //priority is given to the SoftAP interface
};
#endif
// ************ HASH DEFINES *******************


  
/*
* Maps the role Id to role name
* return type : const char*
*/
const char* map_role(short role_type) {
  switch (role_type) {
    case ESP_NOW_ROLE_IDLE:
      return "IDLE";
    case ESP_NOW_ROLE_CONTROLLER:
      return "CONTROLLER";
    case ESP_NOW_ROLE_SLAVE:
      return "SLAVE";
    case ESP_NOW_ROLE_COMBO:
      return "COMBO";
    default:
      return "UNKNOWN";
  }
}

/*
* Gets the WiFi channel of the SSID of your router, It has to be on the same channel as what the Slave is on
* While theritically it should be possible for WiFi and ESPNow to work on different channels but it seems due to some bug (or behavior) this isnt possible with espressif
* return type : uint8_t
*/
uint8_t get_channel_from_ssid(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          DPRINTFLN("Found SSID: %s on Channel %u",WiFi.SSID(i).c_str(),WiFi.channel(i));
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  DPRINTFLN("Could not find the SSID : %s",ssid);
  return 0;
}

/*
* returns the current WiFi channel of the ESP
* return type : uint8_t
*/
uint8_t getWiFiChannel()
{
  #if defined(ESP8266)
    return wifi_get_channel();
  #elif defined(ESP32)
    uint8_t channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&channel,&second);
    return channel;
  #endif
}

void set_wifi_channel(short channel)
{
#if defined(ESP8266)
  wifi_promiscuous_enable(true);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(false);
#elif defined(ESP32)
  esp_wifi_set_promiscuous(true);
  wifi_second_chan_t second;
  esp_wifi_set_channel(channel,second);
  esp_wifi_set_promiscuous(false);
  WiFi.disconnect();
#endif

}

/*
* Determines the right channel for WiFi on the ESP. It finds the channel of the SSID passed as a parameter and returns the channel of the ESP to match the same
* It also stores it in the EEPROM memory if different from the one already on for later use
* While theoritically it should be possible for WiFi and ESPNow to work on different channels but it seems due to some bug (or behavior) this isnt possible with espressif
* paramter- ssid - SSID for which to match the channel
* parameter- forceChannelRefresh - force the code to scan the SSID channel afresh
*/
uint8_t refresh_espnow_channel(const char ssid[MAX_SSID], bool forceChannelRefresh = false)
{
  uint8_t current_channel = 0;
  uint8_t new_channel = 0;
  DPRINTFLN("Current channel:%d",getWiFiChannel());
  
  #if USING(EEPROM_STORE)
  // only get channel from memory if you're not forcing the channel
  if(!forceChannelRefresh)
  {
    // Try to Get the channel from EEPROM
    current_channel = EEPROM.read(0);
    DPRINTFLN("Wifi channel read from memory = %d",current_channel);
  }
  #endif
  
  if((current_channel <= 0 || current_channel > 14) || forceChannelRefresh )//we have an invalid channel, it can only range between 1-14 , scan for a valid channel
  {
    new_channel = get_channel_from_ssid(ssid);
  }
  else
    new_channel = current_channel; // assign the current channel, no changes are needed, this function will exit successfully below
  
  if(new_channel != 0)
  {
    if(new_channel != current_channel)//only write the new channel if it's different from the one we already have to avoid wearing the EEPROM
    {
      #if USING(EEPROM_STORE)
      EEPROM.write(0,(int)new_channel);
      EEPROM.commit();
      // read back the channel to see if it was written properly
      if(EEPROM.read(0) == new_channel)
        {DPRINTFLN("New wifi channel: %d successfully written to memory",new_channel);}
      else
        {DPRINTFLN("Failed to write wifi channel %d to memory",new_channel);}
      #endif
      // Now set the new channel
      set_wifi_channel(new_channel);
      DPRINTFLN("New WiFi channel set as:%d",getWiFiChannel());
    }
    else
      DPRINTFLN("WiFi channel left unchanged to:%d",new_channel);
  }
  else
    DPRINTLN("Failed to get a valid channel for " && ssid);
  //  WiFi.printDiag(Serial);
  return new_channel;
}

/*
* calls all statements to initialize the esp device for espnow messages
* param: channel : the channel number to be set , can be 1-14 only
*/
void initilizeESP(esp_now_role role)
{
  // if we're forcing an init again, deinit first
  esp_now_deinit();
  //Init ESP-NOW
  if (esp_now_init() != 0) {
    DPRINTLN("Error initializing ESP-NOW");
    return;
  }
  DPRINTFLN("WiFi Channel set to %d. Ensure Rx & Tx are on the same channel.",WiFi.channel());
  #if defined(ESP8266)
  esp_now_set_self_role(role);
  #endif
}

/*
* calls all statements to initialize the esp device for espnow messages
* It sets STA mode, sets the channel for wifi/espnow, sets ESP role for ESP8266
* param: channel : the channel number to be set , can be 1-14 only
* param: wifi_mode : WiFi mode to be set , should be WIFI_STA - for sensors , WIFI_AP_STA for Receivers
*/
void initilizeESP(esp_now_role role, WiFiMode_t wifi_mode)
{
  // Set device as a Wi-Fi Station
  WiFi.mode(wifi_mode);
  initilizeESP(role);
}

/*
* calls all statements to initialize the esp device for espnow messages
* It sets STA mode, sets the channel for wifi/espnow, sets ESP role for ESP8266
* param: channel : the channel number to be set , can be 1-14 only
* param: wifi_mode : WiFi mode to be set , should be WIFI_STA - for sensors , WIFI_AP_STA for Receivers
*/
void initilizeESP(uint8_t channel,esp_now_role role, WiFiMode_t wifi_mode)
{
  if(channel < 1 || channel > 14)
  {
    DPRINTLN("Error: Channel No can only be within the range 1-14");
    return;
  }
  set_wifi_channel(channel);
  initilizeESP(role,wifi_mode);
}

/*
* calls all statements to initialize the esp device for espnow messages
* It sets STA mode, reads channel from EEPROM, if invalid, re-scans channel, sets ESP role
* param - ssid - SSID for which the channel has to be matched
* param - esp_now_role - only applicable for ESP8266 - role of the device , NULL for ESP32
* param - restartOnError - bool , to restart ESP if we we're not able to set the channel properly
* param: forceChannelRefresh : true forces the channel to be scanned again , false: tries to read the channel from RTC memory, if it fails then scans afresh
* param: wifi_mode : WiFi mode to be set , should be WIFI_STA - for sensors , WIFI_AP_STA for Receivers
*/
void initilizeESP(const char ssid[MAX_SSID],esp_now_role role, short fallback_channel=DEFAULT_ESPNOW_CHANNEL, bool forceChannelRefresh = false, WiFiMode_t wifi_mode= WIFI_STA)
{
  // Set device WiFi mode
  WiFi.mode(wifi_mode);
  uint8_t ch = refresh_espnow_channel(ssid,forceChannelRefresh);
  if(ch == 0)
    ch = fallback_channel;
  initilizeESP(ch,role,wifi_mode);
}

#if defined(ESP8266)
/*
* Deletes the peer. 
* peer - esp_now_peer_info_t struct containing address of peer eg: {0x05, 0xFF, 0xAF, 0x02, 0xFF, 0x02}
*/
void delete_peer(uint8_t peerAddress[])
{
  //delete peer if it is present
  esp_now_del_peer(peerAddress);//delete peer if it is present
}

/*
* Deletes and re-adds the peer. As peer is channel specific, call this when adding a peer for the first time or when channel number changes
* peerAddress - address of peer, an array of 8 uint8_t elements  eg: {0x05, 0xFF, 0xAF, 0x02, 0xFF, 0x02}
* key - security key - , an array of 16 uint8_t elements OR NULL  eg: {0x33, 0x43, 0x33, 0x14, 0x33, 0x44, 0x33, 0xF4, 0x00, 0x64, 0xA3, 0x24, 0x73, 0x43, 0x3C, 0x55}
* role - role of the peer - type esp_now_role : 1=ESP_NOW_ROLE_CONTROLLER, 2=ESP_NOW_ROLE_SLAVE, 3=ESP_NOW_ROLE_COMBO
*/
bool refreshPeer(uint8_t peerAddress[],const uint8_t key[],esp_now_role role)
{
    delete_peer(peerAddress);//delete peer if it is present
    //Add peer , Note: There is no method in ESP8266 to add a peer by passing esp_now_peer_info_t object unlike ESP32
    uint8_t ch = WiFi.channel();
    if (esp_now_add_peer((uint8_t*)peerAddress, role, ch,(uint8_t*) key, key == NULL ? 0 : KEY_LEN) != 0){
        DPRINTFLN("Failed to add peer on channel:%u",ch);
        return false;
    }
    uint8_t *peerCheck = esp_now_fetch_peer(true);
    if (peerCheck != nullptr)
      {DPRINTF("Added peer: %02X:%02X:%02X:%02X:%02X:%02X on channel:%u",peerCheck[0],peerCheck[1],peerCheck[2],peerCheck[3],peerCheck[4],peerCheck[5],ch);
        DPRINTFLN(" with role:%s",map_role(role));
      }
    else
      {DPRINTLN("Failed to set the peer");}
    return true;
}
#elif defined(ESP32)

/*
* Deletes the peer. 
* peer - esp_now_peer_info_t struct containing address of peer eg: {0x05, 0xFF, 0xAF, 0x02, 0xFF, 0x02}
*/
void delete_peer(esp_now_peer_info_t *peer)
{
  //delete peer if it is present
  if(esp_now_is_peer_exist(peer->peer_addr))
    esp_now_del_peer(peer->peer_addr);
}
/*
* Deletes and re-adds the peer. As peer is channel specific, call this when adding a peer for the first time or when channel number changes
* peer - esp_now_peer_info_t struct containing address of peer eg: {0x05, 0xFF, 0xAF, 0x02, 0xFF, 0x02} and other details
*/
bool refreshPeer(esp_now_peer_info_t *peer)
{
    delete_peer(peer);
    
    // Set the right channel of the peer
    peer->channel = WiFi.channel();
    // Register the peer
    if (esp_now_add_peer(peer) != ESP_OK)
    {
        DPRINTFLN("Failed to add peer on channel:%u",peer->channel);
        return false;
    }
    else
      {DPRINTFLN("Added peer: %02X:%02X:%02X:%02X:%02X:%02X on channel:%u",peer->peer_addr[0],peer->peer_addr[1],peer->peer_addr[2],peer->peer_addr[3],peer->peer_addr[4],peer->peer_addr[5],peer->channel);}
    
    return true;
}

#endif

/*
* Sends a espnow_message to the slave. It retries to send the message a certain no of times if it fails and then gives up
* Relies on the variable bResultReady & deliverySuccess which is set to true in the call back function of the OnDataSent to determine if the
   message sending was successful. You must set these in the OnDataSent function in your code
*/
bool sendESPnowMessage(espnow_message *myData,uint8_t peerAddress[], short retries=1,bool ack= true)
{
  bResultReady = false;
  // retries should at least be 1 when acknowledgement is required so that a message is tried twice in the loop, this is so that if channel number needs refreshed,
  // message sending is tried again.
  if(retries<1 && ack)
    retries = 1;
  // try to send the message MAX_MESSAGE_RETRIES times if it fails
  for(short i = 0;i<=retries;i++)
  {
    int result = esp_now_send(peerAddress, (uint8_t *) myData, sizeof(*myData));
    if (result == 0)
      {DPRINTLN("Sent message, waiting for delivery...");}
    else
      {DPRINTFLN("Error sending the message, error code: %d",result);}

    // if(result == ESP_ERR_ESPNOW_NOT_INIT)
    //   DPRINT("ESP not init");

    if(ack)
    {
      long waitTimeStart = millis();
      //get a confirmation of successful delivery , else try again. This flag is set in the callback OnDataSent from the calling code
      while(!bResultReady && ((millis() - waitTimeStart) < WAIT_TIMEOUT))
      {
        delay(1);
      }
      //DPRINTFLN("wait:%u",(millis() - waitTimeStart));
      if(result == 0  && deliverySuccess == 0)
      {
        break;
      }
      else
      {
          bResultReady = false; // message sending failed , prepare for next iteration
          // // See if we are on the right channel, it might have changed since last time we wrote the same in EEPROM memory
          // // Do it only once in the cycle to send a message else it consumes battery every time the ESP tries to send in case
          // // there is permanent error in sending a message - eg. in case the Slave isnt available
          #if USING(EEPROM_STORE)
          if(!channelRefreshed)
          {
              uint8_t ch = refresh_espnow_channel(ssid,true); //force the channel refresh
              if(ch != 0)
                set_wifi_channel(ch);
              channelRefreshed = true;// this will enable refreshing of channel only once in a cycle, unless the flag is again reset by the calling code
          }
          #endif
      }
    }
    else
      return result==0?true:false;
  }
  return deliverySuccess;
}

/*
* sets the custom STA or Soft AP MAC address for an ESP , compatible with ESP8266/ESP32
* input param STA_MAC : bool , True for STA_MAC , false for Soft AP MAC
* return type : bool , true if successfully set else false
*/
bool setCustomMAC(uint8_t customMACAddress[6], bool STA_MAC)
{
  #if defined(ESP8266)
  uint8 mac_type;
  if(STA_MAC)
  {
    mac_type = STATION_IF;
    WiFi.mode(WIFI_STA);// This has to be set before you attempt to change the MAC address in station mode, else it fails
  }
  else
    mac_type = SOFTAP_IF; // WiFi mode not necessary for this
  if(wifi_set_macaddr(mac_type, &customMACAddress[0]))
    { 
      uint8_t macAddr[6];
      WiFi.softAPmacAddress(macAddr);
      DPRINT("Successfully set a custom MAC address as:"); 
      DPRINTLN((STA_MAC?WiFi.macAddress():WiFi.softAPmacAddress()));
      return true;
    }
  else
    {DPRINT("Failed to set custom MAC address. Current MAC address is:"); DPRINTLN(WiFi.softAPmacAddress());}
  #elif defined(ESP32)
  wifi_interface_t mac_type;
  if(STA_MAC)
  {
    mac_type = WIFI_IF_STA;
    WiFi.mode(WIFI_STA);// This has to be set before you attempt to change the MAC address in station mode, else it fails
  }
  else
    mac_type = WIFI_IF_AP; // WiFi mode not necessary for this
  if(esp_wifi_set_mac(mac_type, &customMACAddress[0]))
    { 
      DPRINT("Successfully set a custom MAC address as:");
      DPRINTLN((STA_MAC?WiFi.macAddress():WiFi.softAPmacAddress()));
      return true;
    }
  else
    {DPRINT("Failed to set custom MAC address. Currenct MAC address is:"); DPRINTLN(WiFi.softAPmacAddress());}
  #endif
  return false;
}



