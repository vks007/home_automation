#include <HCSR04.h>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

#define Rx D8 // D2 for ESP //not used , dummy
#define Tx D0 //D1 for ESP 

RH_ASK driver(2000, Rx, Tx, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 

// UltraSonicDistanceSensor(Trigger pin , echo pin)
UltraSonicDistanceSensor sonar(D1, D2);

struct SensorData { 
  int height;
};

SensorData sensor; 

void setup () {
    Serial.begin(115200);  // We initialize serial connection so that we could print values from sensor.
  if(driver.init())
    Serial.println("Driver initialized");
}

void loop () {
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    uint8_t distance = sonar.measureDistanceCm();
    Serial.println(distance);
    sensor.height = distance;
    //uint8_t distance = 1234;
    driver.send((uint8_t*)&sensor, sizeof(sensor));
    driver.waitPacketSent();
    
    delay(500);
}
