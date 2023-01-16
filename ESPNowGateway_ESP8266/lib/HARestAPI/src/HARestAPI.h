#ifndef HARestAPI_h
#define HARestAPI_h

#include <Arduino.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#ifdef ESP8266
#include <ESP8266HTTPClient.h>
#else
#include <HTTPClient.h>
#endif

class HARestAPI
{
public:
	HARestAPI(WiFiClient &client);
	HARestAPI(WiFiClientSecure &client);
	void setHAServer(String);
	void setHAServer(String, uint16_t);
	void setHAServer(String, uint16_t, String);
	void setHAPassword(String);
	void setFingerPrint(String);
	void setDebugMode(bool);
	void setTimeOut(uint16_t);
	void setURL(String);
	bool sendCustomHAData(String, String);
	bool sendHA(void);
	bool sendHAURL(String);
	String sendGetHA(String);
	String sendGetHA(void);
	bool sendPostHA(String);
	bool sendPostHA(String, String);

private:
	WiFiClient *wclient = nullptr;
	WiFiClientSecure *wsclient = nullptr;
	HTTPClient *http = nullptr;

	String
		_serverip,
		_password,
		_url = "/api/services/api/",
		_component,
		_fingerprint;

	uint16_t
		_port,
		_time_out = 0;

	bool
		_debug = true,
		_ssl = false,
		_skip_sendurl = false;
};

#endif