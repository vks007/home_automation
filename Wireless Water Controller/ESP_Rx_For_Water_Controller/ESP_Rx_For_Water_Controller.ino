// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif


//Format for RH_ASK params is : speed, Rx pin , Tx pin
//RH_ASK driver;
 RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2 , GPIO4 - Rx , GPIO5 - Tx
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 

struct SensorData { 
  int height;
};

SensorData sensor;

void setup()
{
    Serial.begin(115200);
    Serial.println("");
    
    if (!driver.init())
      Serial.println("failed to initialise driver");
    else
      Serial.println("driver initialised");
}

void loop()
{
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      // Message with a good checksum received, dump it.
      //driver.printBuffer("Got:", buf, buflen);
      //String str = (char*)buf;
      //Serial.println(str);
      memcpy(&sensor, &buf, sizeof(sensor));
      Serial.println(sensor.height);
    }
}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}
