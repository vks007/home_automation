// ---------------------------------------------------------------------------
// Ensure #define TIMER_ENABLED is true in NewPing.h  for use with ESP8266
// ---------------------------------------------------------------------------
#include <NewPing.h> 
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

#define TRIGGER_PIN D0
#define ECHO_PIN D5
#define DISTANCE 300

#define Rx D2 // D2 for ESP
#define Tx D1 //D1 for ESP 

RH_ASK driver(2000, Rx, Tx, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, DISTANCE); 

struct SensorData { 
  int height;
};

SensorData sensor; 

void setup() {
  Serial.begin(115200);
  if(driver.init())
    Serial.println("Driver initialized");
}
void loop() {
  uint8_t distance = sonar.ping_cm();
  //Serial.println(distance);
  sensor.height = distance;
//uint8_t distance = 1234;
  driver.send((uint8_t*)&sensor, sizeof(sensor));
    
  //char msg[4];
  //itoa(distance,msg,10);
  //driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  delay(1000);
}
