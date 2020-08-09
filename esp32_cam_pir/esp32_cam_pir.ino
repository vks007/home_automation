/*********
  References:
  https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  https://github.com/espressif/esp-who/issues/90 - talks about an issue where you cant use attachInterrupt with esp32 cam

  Dependencies:
  PubSubClient - for MQTT
  
  UPLOAD SETTINGS!!! 
   Board : "ESP32 Wrower Module"
   Partition Scheme: Huge App (3MB no OTA 1MB SPIFFS) , Dont use OTA with this, it doesnt work. Also the camera is slow due to lack of space
  
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
#include <PubSubClient.h>
#define DEBUG //BEAWARE that this statement should be before #include <DebugUtils.h> else the macros wont work as they are based on this #define
#include "Debugutils.h" //This file is located in the Sketches\libraries\DebugUtils folder

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

// MQTT related info
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!             ATTENTION                       !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//Will have to see if I can afford to wait for this long a time before retrying the MQTT connection. It might trigger a WDT somewhere or fail the ESP32 CAM code
#define MAX_MQTT_CONNECT_RETRY 4 //max no of retries to connect to MQTT server
#define CONNECTION_RETRY_TIME 500 //time in ms to wait before retrying MQTT connection
#define MQTT_USER MQTT_USER1
#define MQTT_PASSWORD MQTT_PSWD1
#define MQTT_HOST MQTT_SERVER1
#define MQTT_PORT MQTT_PORT1
#define MQTT_PIR_TOPIC "home/camera1/motion"
#define MQTT_WIFI_TOPIC "home/camera1/wifi"
#define MSG_ON "on" //payload for ON
#define MSG_OFF "off"//payload for OFF
#define ESP_IP_ADDRESS          IPAddress(192,168,1,62)
#define VERSION "1.0.1"
#include "version.h" 

const char* ssid = SSID2;
const char* password = SSID2_PSWD;
const char* deviceName = "ESP32_CAM_PIR";

WiFiClient espClient;
PubSubClient client(espClient);

#define MOTION_WAIT_TIME 5 //Time to wait for motion trigger before publishing motion OFF message
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
      DPRINTLN("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            DPRINTLN("JPEG compression failed");
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

void publishMessage(const char msg[], const char topic[], bool retained = false) {
  // Loop until we're reconnected
  char i = 0;
  while (!client.connected()) 
  {
    i++;
    DPRINT("Attempting MQTT connection...");
    // Attempt to connect
    client.connect(deviceName,MQTT_USER,MQTT_PASSWORD);
    if(i >= MAX_MQTT_CONNECT_RETRY)
      break;
  }

  if (client.connected()) {
    DPRINTLN("connected");
    client.publish(topic, msg,retained);
    DPRINT("published message:");
    DPRINTLN(msg);
  } 
  else 
  {
    DPRINT("failed, rc=");
    DPRINT(client.state());
    DPRINT(" try again in ");
    DPRINT(CONNECTION_RETRY_TIME);
    DPRINTLN(" ms");
    delay(CONNECTION_RETRY_TIME);
  }
}


// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR onMotionDetect(void* arg) {
//  DPRINTLN("MOTION DETECTED!!!");
  motionDetected = true;
  startTimer = true;
  lastTrigger = millis();
}



void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  DBEGIN(115200);
  //Serial.setDebugOutput(false);
  
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
  
  if (!WiFi.config(ESP_IP_ADDRESS, GATEWAY1, SUBNET1)) {
    DPRINTLN("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DPRINT(".");
  }
  DPRINTLN("");
  DPRINTLN("WiFi connected");

  // Start streaming web server
  startCameraServer();
  DPRINT("Camera Stream Ready! Go to: http://");
  DPRINTLN(WiFi.localIP());

  //Instantiate the MQTT server
  client.setServer(MQTT_SERVER1, MQTT_PORT1);

  String str = "{\"ip_address\":\"" + WiFi.localIP().toString() + "\" , \"mac\":\"" + WiFi.macAddress() + "\" , \"version\": \"" + compile_version + "\"}";
  
  publishMessage(str.c_str(),MQTT_WIFI_TOPIC,true);

}

void loop() {
  // Current time
  now = millis();
  
  if(motionDetected)
  {
      publishMessage(MSG_ON,MQTT_PIR_TOPIC);
      motionDetected = false;
      //DPRINTLN("Publishing motion on");
  }
  if(startTimer && (now - lastTrigger > (MOTION_WAIT_TIME*1000)))
  {
    DPRINTLN("Motion stopped...");
    publishMessage(MSG_OFF,MQTT_PIR_TOPIC);
    //DPRINTLN("Publishing motion off");
    startTimer = false;
  }
}
