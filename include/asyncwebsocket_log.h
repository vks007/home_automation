#ifndef WEBSOCKET_LOGS
#define WEBSOCKET_LOGS
/*
 * This file provides a web socket capability to any ESP based program. It gives the ability to look at progress message logs via a web browser instead of serial monitor
 * To use this follow the following steps

	#define WEBSOCKETS IN_USE // or NOT_IN_USE
	#include "websocket_log.h"
	
	call the following code in setup()
	  WS_SERVER_SETUP()
	  server.begin(); // This needs to be called only if you're not already using a server for something else
	  WS_SETUP();
	  
	call WS_LOOP() in loop()
	Then call WS_BROADCAST_TXT with a string argument of what needs to be displayed eg. String msg = "test message"; WS_BROADCAST_TXT(msg);
	To see the messages Go to the http://<ESP IP address>/logs on your browser to see the messages eg. 192.168.1.2:/logs
*/

	#include "macros.h"

	#if USING(WEBSOCKETS)
	#define WS_SETUP() ws_setup()

  
	  
	#include <ESPAsyncWebServer.h>
	AsyncWebSocket ws("/log");

	void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
		if(type == WS_EVT_CONNECT){
			Serial.println("Websocket client connection received");
		} else if(type == WS_EVT_DISCONNECT){
			Serial.println("Client disconnected");
		} else if(type == WS_EVT_DATA){
			Serial.println("Data received: ");
			for(int i=0; i < len; i++) {
			Serial.print(data[i]);
			Serial.print("|");
			}
			Serial.println();
		}
	}

	void ws_setup()
	{
	  ws.onEvent(onWsEvent);
	}

	#else
	#define WS_SETUP()

	#endif


#endif
