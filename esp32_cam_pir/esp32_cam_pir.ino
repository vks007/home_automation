/*********
  References:
  https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  Example mqtt code from examples in https://github.com/marvinroger/async-mqtt-client
  async mqtt api reference: http://marvinroger.viewdocs.io/async-mqtt-client/2.-API-reference/
  https://github.com/espressif/esp-who/issues/90 - talks about an issue where you cant use attachInterrupt with esp32 cam

  Dependencies:
  AsyncTCP - download from here https://github.com/me-no-dev/AsyncTCP (Note: this is for ESP32 , there is a different one for ESP8266
  async-mqtt-client - download from here : https://github.com/marvinroger/async-mqtt-client
  
  UPLOAD SETTINGS!!! 
   Board : "ESP32 Wrower Module"
   Partition Scheme: Minimal SPIFFS (1.9MB App with OTA/190KB SPIFFS)
  
*********/
/*
 *  This is an example for ESP32 CAM which shows a webpage where you can select the picture properties , get a still image or stream a video stream
 * Settings to use : Module : ESP Wrover Module w/ Partition Scheme as 3MB without OTA 
 * I had trouble with making the camera work with static IP settings. For some reason code copied form ESP8266 example did not work. Make sure you take the code
 * as written in this example of you're setting static IP. for me I was getting time out on the wen page if did the static IP the ESP8266 way although IP was assinged to 
 * ESP32 properly
*/


#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include "secrets.h" //From /libraries/MyFiles/secrets.h
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

//Replace with your network credentials
const char* ssid = SSID1;
const char* password = SSID1_PSWD;
const char* deviceName = "ESP32_CAM_PIR";

// MQTT related info
#define MQTT_USER MQTT_USER1
#define MQTT_PASSWORD MQTT_PSWD1
#define MQTT_HOST MQTT_SERVER1
#define MQTT_PORT MQTT_PORT1
#define MQTT_TOPIC "home/camera1/motion"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 62);
IPAddress gateway(192, 168, 1, 1); // Set your Gateway IP address
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

#define MOTION_WAIT_TIME 10 //Time to wait for motion trigger before publishing motion OFF message
#define PIR_SENSOR_PIN GPIO_NUM_13
// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean motionDetected = false;
boolean startTimer = false;

#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

/*
Framesize can be one of: 
    FRAMESIZE_96x96,    // 96x96
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QQVGA2,   // 128x160
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_240x240,  // 240x240
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    FRAMESIZE_QXGA,     // 2048*1536
*/    
#define FRAME_SIZE FRAMESIZE_UXGA //This is the max my camera support, dont go beyond this. I get error timeout waiting fro VSYNC above this.
//#define FRAME_SIZE FRAMESIZE_VGA // Use this for testing as image vidoe is much faster at this frame size


#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  /*
  //Subscribe to a topic
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);

  //Publish a message at QoS 1
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  */
  
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged. | ");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR onMotionDetect(void* arg) {
  Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
  startTimer = true;
  lastTrigger = millis();
}


void setupOTA()
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  //ArduinoOTA.setHostname("esp32_camera");

  // No authentication by default
   ArduinoOTA.setPassword(OTA_PSWD);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("");
  Serial.println("OTA initialized");
  
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    config.frame_size = FRAME_SIZE;//FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAME_SIZE;//FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(PIR_SENSOR_PIN, INPUT_PULLUP);
  
  /*You cant use attachInterrupt(digitalPinToInterrupt(PIR_SENSOR_PIN), onMotionDetect, RISING);
   * This is because esp_camera_init already calls the isr initialization code and attachInterrupt calls it again so it crashes
   * See here for details https://github.com/espressif/esp-who/issues/90
   * Ensure the below code to attach interrupt to PIR PIN is called after esp_camera_init
  */
  err = gpio_isr_handler_add(PIR_SENSOR_PIN, &onMotionDetect, (void *) 13);
  if (err != ESP_OK) {
  Serial.printf("handler add failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(PIR_SENSOR_PIN, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
  Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  
  // Wi-Fi connection
  WiFi.setHostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

//  setupOTA();
  
  // Start streaming web server
  startCameraServer();
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());

  //Set up MQTT related params
  mqttClient.setCredentials(MQTT_USER,MQTT_PASSWORD);
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  //Set up MQTT related params end

  connectToMqtt(); //connect to MQTT
  
}

void loop() {
//  ArduinoOTA.handle();
  // Current time
  now = millis();
  
  if(motionDetected)
  {
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_TOPIC, 1, true, "on");
      motionDetected = false;
      Serial.print("Publishing motion on at QoS 1, packetId: ");
      Serial.println(packetIdPub1);
  }
  if(startTimer && (now - lastTrigger > (MOTION_WAIT_TIME*1000)))
  {
    Serial.println("Motion stopped...");
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_TOPIC, 1, true, "off");
    Serial.print("Publishing motion off at QoS 1, packetId: ");
    Serial.println(packetIdPub1);
    startTimer = false;
  }
}
