/*
Utility file which encapsulates the functionality of initializing espnow on a controller (aka sensor). Has the following features
* Scans the WiFi channel for the peer using SSID and stores it in EEPROM for later use. This saves ~2 sec when sending messages
* Retries the sending of a failed message a certain no of times (passed as paramter)
* Retries refreshing of WiFi channel no if sending fails only once, if its required to scan the channel again, call setChannelRefreshFlag)(false)
* Initilizes the espnow for espnow functions , sets role etc
* refreshes the peer, required in case channel changes
* sends a message via espnow
* usage : 
  - instantiate an object of esputil passing in the role & SSID in the constructor
  - call inilize() and then refreshPeer()
  - send message via sendMessage() and monitor the result of the message send via the flag bResult
  - set the flag bResultReady in OnDataSent when a confirmation of the same is received

*/

#ifndef ESPNOW_CONTROLLER_H
#define ESPNOW_CONTROLLER_H
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "espnowMessage.h" // for struct of espnow message
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder
#include <ESP_EEPROM.h> // to store the WiFi channel number in EEPROM for faster adding of peer next time on wards

#define KEY_LEN  16 // lenght of PMK & LMK key (fixed at 16 for ESP)
#define WAIT_TIMEOUT 25 // time in millis to wait for acknowledgement of the message sent
#define CONNECTION_RETRY_INTERVAL 30 // time is secs to wait before refreshing the connection in case of failure
#define MAX_SSID_LEN 50
//Define the esp_now_peer_info if we're working with esp8266, for ESP32 its already defined
#ifdef ESP8266
typedef struct esp_now_peer_info {// this is defined in esp_now.h for ESP32 but not in espnow.h for ESP8266
  u8 peer_addr[6];
  uint8_t channel;
  uint8_t encrypt;
}esp_now_peer_info_t;
#endif

// enum	esp_now_role	{
// 	 ESP_NOW_ROLE_IDLE	=	0, // no interface
// 	 ESP_NOW_ROLE_CONTROLLER, //priority is given to the station interface
// 	 ESP_NOW_ROLE_SLAVE, //priority is given to the SoftAP interface
// 	 ESP_NOW_ROLE_COMBO,	 //	priority is given to the station interface
// 	 ESP_NOW_ROLE_MAX, //priority is given to the SoftAP interface
// };

class esputil
{
  private:
  esputil(){} //disable default constructor
  uint8_t _channel = 0;//stores the channel of the slave by scanning the SSID the slave is on.
  bool channelRefreshed = false;//tracks the status of the change in wifi channel , true -> wifi channel has been refreshed
  // it is set to false every time the ESP boots
  esp_now_role _role;
  char _ssid[MAX_SSID_LEN];
  
  /*
  * Gets the WiFi channel of the SSID of your router, It has to be on the same channel as this one as the receiver will listen to ESPNow messages on the same 
  * While theritically it should be possible for WiFi and ESPNow to work on different channels but it seems due to some bug (or behavior) this isnt possible with espressif
  */
  uint8_t getWiFiChannel(const char *ssid) {
    if (int32_t n = WiFi.scanNetworks()) {
        for (uint8_t i=0; i<n; i++) {
            DPRINTLN(WiFi.SSID(i).c_str());
            if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
                return WiFi.channel(i);
            }
        }
    }
    return 0;
  }

  /*
  * Finds the right channel as per the router's WiFi channel and stores it in the EEPROM memory if different from the one already on. 
  * While theritically it should be possible for WiFi and ESPNow to work on different channels but it seems due to some bug (or behavior) this isnt possible with espressif
  */
  void setSSIDChannel(bool forceChannelRefresh = false, bool restartOnError= false)
  {
    DPRINTFLN("Current channel:%d",wifi_get_channel());
    if(_channel == 0) // we're starting up , get the channel from the EEPROM
    {
      if(EEPROM.get(0, _channel))
      {DPRINTFLN("wifi channel read from memory = %d",_channel);}
      else
      {DPRINTFLN("Failed to read wifi channel from memory:%d",_channel);}
    }
    
    if((_channel <= 0 || _channel > 14) || forceChannelRefresh )//we have an invalid channel, it can only range between 1-14 , scan for a valid channel
    {
        DPRINTFLN("Scanning channel for SSID = %s",_ssid);
        uint8_t new_channel = getWiFiChannel(_ssid);
        DPRINTFLN("new wifi channel scanned = %d",new_channel);
        if(new_channel != 0)
        {
          if(new_channel != _channel)//only write the new channel if its different from the one we already have
          {
            _channel = new_channel;
            if(EEPROM.put(0, _channel) && EEPROM.commit())
              {DPRINTFLN("new wifi channel written to memory = %d",_channel);}
            else
              {DPRINTFLN("Failed to write wifi channel %d to memory",_channel);}
          }
        }
        else
          DPRINTLN("Failed to get a valid channel for " && _ssid);
    }
    // sometimes we get a channel 0 after scanning, that's because at times the SSID we're scanning for isnt available. I have seen this happen many times.
    //WiFi.disconnect();
    //  WiFi.printDiag(Serial);
    // only change channel if its different from the earlier one
    // To change the channel you have to first call wifi_promiscuous_enable(true) , change channel and then call wifi_promiscuous_enable(false)
    if((wifi_get_channel() != _channel) && _channel !=0)// no use changing channel if we got 0, it can happen if you dont find the SSID
    {
      wifi_promiscuous_enable(true);
      wifi_set_channel(_channel);
      wifi_promiscuous_enable(false);
      delay(10);// not sure why did I keep this delay, may remove it later
    }
    uint8_t ch = wifi_get_channel();
    DPRINTFLN("New channel:%d",ch);
    //strange behavior : If I define ch as byte and make the comparison below , the ESP resets due to WDT and then hangs
    if(ch == 0 && restartOnError)
    {
      DPRINTLN("Could not set WiFi Channel properly, restarting");
      ESP.restart();
    }
    //  WiFi.printDiag(Serial);
  }

  public:
  esputil(esp_now_role role,const char ssid[])
  {
    _role = role;
    //memcpy(_ssid,ssid,sizeof(ssid)>MAX_SSID_LEN?MAX_SSID_LEN:sizeof(ssid));//max size of ssid string is 50
    memcpy(_ssid,"EAGLE_EXT",sizeof("EAGLE_EXT"));//max size of ssid string is 50
  }
  volatile uint8_t deliverySuccess = 9; //0 means success , non zero are error codes
  volatile bool bResultReady = false;
  
  void setChannelRefreshFlag(bool flag)
  {
    channelRefreshed = flag;
  }

  /*
  * calls all statements to initialize the esp device for espnow messages
  * It sets STA mode, reads channel from RTC, if invalid, re-scans channel, sets ESP role
  * Params: forceChannelRefresh : true forces the channel to be scanned again , false: tries to read the channel from RTC memory, if it fails then scans afresh
  */
  void initilize(bool forceChannelRefresh = false, bool restartOnError= false)
  {
    setSSIDChannel(forceChannelRefresh);
    
    // Set device as a Wi-Fi Station and set channel
    WiFi.mode(WIFI_STA);
    // if we're forcing an init again, deinit first
    esp_now_deinit();
    //Init ESP-NOW
    if (esp_now_init() != 0) {
      DPRINTLN("Error initializing ESP-NOW");
      return;
    }
    esp_now_set_self_role(_role);
  }

  /*
  * Deletes and re-adds the peer , Required to be called when the WiFi channel number changes
  */
  bool refreshPeer(uint8_t peerAddress[],const uint8_t key[])
  {
      esp_now_del_peer(gatewayAddress);//delete peer if it is present
      //Add peer , Note: There is no method in ESP8266 to add a peer by passing esp_now_peer_info_t object unlike ESP32
      if (esp_now_add_peer((u8*)peerAddress, ESP_NOW_ROLE_SLAVE, _channel,(uint8_t*) key, key == nullptr ? 0 : KEY_LEN) != 0){
//      if (esp_now_add_peer((u8*)peerAddress, ESP_NOW_ROLE_SLAVE, _channel,(uint8_t*) NULL, 0) != 0){
          DPRINTFLN("Failed to add peer on channel:%u",_channel);
          return false;
      }
      uint8_t *peerCheck = esp_now_fetch_peer(true);
      
      if (peerCheck != nullptr)
        {DPRINTFLN("Added peer: %X:%X:%X:%X:%X:%X on channel:%u",peerCheck[0],peerCheck[1],peerCheck[2],peerCheck[3],peerCheck[4],peerCheck[5],_channel);}
      else
        {DPRINTLN("Failed to set the peer");}
      return true;
  }

  /*
  * Sends a espnow_message to the slave. It retries to send the message a certain no of times if if fails and then gives up
  * Relies on the variable bResultReady to set true in the call back function of the OnDataSent to determine if the message sending was successful
  * You must set this in the OnDataSent function in your code
  */
  int sendMessage(espnow_message *myData,uint8_t peerAddress[], short retries=1)
  {
    bResultReady = false;
    // retries should at least be 1 so that a message is tried twice in the loop, this is so that if channel number needs refreshed,
    // message sending is tried again.
    if(retries<1)
      retries = 1;
    // try to send the message MAX_MESSAGE_RETRIES times if it fails
    for(short i = 0;i<=retries;i++)
    {
      //DPRINTLN(millis());
      DPRINTLN(myData->intvalue3);
      int result = esp_now_send(peerAddress, (uint8_t *) myData, sizeof(*myData));
      // If devicename is not given then generate one from MAC address stripping off the colon
      long waitTimeStart = millis();
      if (result == 0) DPRINTLN("Sent message, waiting for delivery...");
      else DPRINTLN("Error sending the message");
      
      //get a confirmation of successful delivery , else may try again. This flag is set in the callback OnDataSent from the calling code
      while(!bResultReady && ((millis() - waitTimeStart) < WAIT_TIMEOUT))
      {
        delay(1);
      }
      //DPRINTFLN("wait:%u",(millis() - waitTimeStart));
      if(deliverySuccess == 0)
      {
        break;
      }
      else
      {
          bResultReady = false; // message sending failed , prepare for next iteration
          // //See if we are on the right channel, it might have changed since last time we wrote the same in RTC memory
          // // Scan for the WiFi channel again and store the new value in the RTC memory, Do it only once
          if(!channelRefreshed)
          {
              DPRINTLN("Refresh wifi channel...");
              setSSIDChannel();
              channelRefreshed = true;// this will enable refreshing of channel only once in a cycle, unless the flag is again reset by calling code
          }
      }
    }
    return deliverySuccess;
  }
};

#endif