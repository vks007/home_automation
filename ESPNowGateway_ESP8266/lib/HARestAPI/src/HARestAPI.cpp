#include "HARestAPI.h"


HARestAPI::HARestAPI(WiFiClient &client)
{
  this->wclient = &client;
}

HARestAPI::HARestAPI(WiFiClientSecure &client)
{
  _ssl = true;
  this->wsclient = &client;
}

void HARestAPI::setHAServer(String HAServer)
{
  _serverip = HAServer;
  if (_ssl)
    _port = 443;
  else
    _port = 8123;
}

void HARestAPI::setHAServer(String HAServer, uint16_t Port)
{
  _serverip = HAServer;
  _port = Port;
}

void HARestAPI::setHAServer(String HAServer, uint16_t Port, String Password)
{
  _serverip = HAServer;
  _port = Port;
  _password = Password;
}

void HARestAPI::setHAPassword(String Password)
{
  _password = Password;
}

void HARestAPI::setDebugMode(bool Debug)
{
  _debug = Debug;
}

void HARestAPI::setTimeOut(uint16_t TimeOut)
{
  _time_out = TimeOut;
}


void HARestAPI::setURL(String URL)
{
  _url = URL;
}

void HARestAPI::setFingerPrint(String FingerPrint)
{
  _fingerprint = FingerPrint;
}

String HARestAPI::sendGetHA(String URL)
{
  String posturl, replystr;
  this->http = new HTTPClient();

  if (_ssl)
  {
    posturl = "https://" + _serverip + ":" + _port + URL;
    if (_debug)
    {
      Serial.print("Connecting: ");
      Serial.println(posturl);
    }
    if ( _fingerprint.length() > 0)
    {
      if (1==1) //(wsclient->verify(_fingerprint.c_str(), _serverip.c_str()))
      {
        if (_debug)
          Serial.println("Certificate matches");
      }
      else
      {
        if (_debug)
          Serial.println("Certificate doesn't match");
        _skip_sendurl = true;
      }
    }
    if (!_skip_sendurl)
    {
      http->begin(*wsclient, posturl);
      if (_time_out)
        http->setTimeout(_time_out);
      http->addHeader("User-Agent", "HA Rest API Client");
      http->addHeader("Accept", "*/*");
      http->addHeader("Authorization", "Bearer " + _password);
      int httpCode = http->GET();
      if (httpCode > 0)
      {
        if (_debug)
          Serial.printf("[HTTPS] GET... code: %d\n", httpCode);
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
        {
          replystr += http->getString();
        }
      }
      else
      {
        if (_debug)
          Serial.printf("[HTTPS] GET... failed, error: %s\n", http->errorToString(httpCode).c_str());
      }
      http->end();
    }
    _skip_sendurl = false;
  }
  else
  {
    posturl = "http://" + _serverip + ":" + _port + URL;
    if (_debug)
    {
      Serial.print("Connecting: ");
      Serial.println(posturl);
    }
    http->begin(*wclient, posturl);
    if (_time_out)
      http->setTimeout(_time_out);
    http->addHeader("User-Agent", "HA Rest API Client");
    http->addHeader("Accept", "*/*");
    http->addHeader("Authorization", "Bearer " + _password);
    int httpCode = http->GET();
    if (httpCode > 0)
    {
      if (_debug)
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      {
        replystr += http->getString();
      }
    }
    else
    {
      if (_debug)
        Serial.printf("[HTTP] GET... failed, error: %s\n", http->errorToString(httpCode).c_str());
    }
    http->end();
  }

  if (_debug)
  {
    Serial.println("reply was:");
    Serial.println("==========");
    Serial.println(replystr);
    Serial.println("==========");
    Serial.println("closing connection");
  }
  delete http;
  return replystr;
}

bool HARestAPI::sendPostHA(String URL, String message)
{
  bool reply = false;
  String posturl, replystr;
  this->http = new HTTPClient();

  if (_ssl)
  {
    posturl = "https://" + _serverip + ":" + _port + URL;
    if (_debug)
    {
      Serial.print("Connecting: ");
      Serial.println(posturl);
    }
    if ( _fingerprint.length() > 0)
    {
      if (1==1 ) //(wsclient->verify(_fingerprint.c_str(), _serverip.c_str()))
      {
        if (_debug)
          Serial.println("Certificate matches");
      }
      else
      {
        if (_debug)
          Serial.println("Certificate doesn't match");
        _skip_sendurl = true;
      }
    }
    if (!_skip_sendurl)
    {
      http->begin(*wsclient, posturl);
      if (_time_out)
        http->setTimeout(_time_out);
      http->addHeader("User-Agent", "HA Rest API Client");
      http->addHeader("Accept", "*/*");
      http->addHeader("Authorization", "Bearer " + _password);
      int httpCode = http->POST(message);
      if (httpCode > 0)
      {
        if (_debug)
          Serial.printf("[HTTPS] POST... code: %d\n", httpCode);
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
        {
          replystr += http->getString();
          reply = true;
        }
      }
      else
      {
        if (_debug)
          Serial.printf("[HTTPS] POST... failed, error: %s\n", http->errorToString(httpCode).c_str());
        reply = false;
      }
      http->end();
    }
    _skip_sendurl = false;
  }
  else
  {
    posturl = "http://" + _serverip + ":" + _port + URL;
    if (_debug)
    {
      Serial.print("Connecting: ");
      Serial.println(posturl);
    }
    http->begin(*wclient, posturl);
    if (_time_out)
      http->setTimeout(_time_out);
    http->addHeader("User-Agent", "HA Rest API Client");
    http->addHeader("Accept", "*/*");
    http->addHeader("Authorization", "Bearer " + _password);
    int httpCode = http->POST(message);
    if (httpCode > 0)
    {
      if (_debug)
        Serial.printf("[HTTP] POST... code: %d\n", httpCode);
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      {
        replystr += http->getString();
        reply = true;
      }
    }
    else
    {
      if (_debug)
        Serial.printf("[HTTP] POST... failed, error: %s\n", http->errorToString(httpCode).c_str());
      reply = false;
    }
    http->end();
  }
  if (_debug)
  {
    Serial.println("reply was:");
    Serial.println("==========");
    Serial.println(replystr);
    Serial.println("==========");
    Serial.println("closing connection");
  }
  delete http;
  return reply;
}

bool HARestAPI::sendPostHA(String message)
{
  return sendPostHA(_url, message);
}

String HARestAPI::sendGetHA(void)
{
  return sendGetHA(_url);
}

bool HARestAPI::sendCustomHAData(String URL, String Message)
{
  return sendPostHA(URL, Message);
}

bool HARestAPI::sendHA(void)
{
  String Message = "{\"entity_id\":\"" + _component + "\"}";
  return sendPostHA(_url, Message);
}


bool HARestAPI::sendHAURL(String URL)
{
  String Message = "{\"entity_id\":\"" + _component + "\"}";
  return sendPostHA(URL, Message);
}


