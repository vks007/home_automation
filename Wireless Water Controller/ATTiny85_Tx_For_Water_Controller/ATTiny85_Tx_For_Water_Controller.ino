// ---------------------------------------------------------------------------
// #define TIMER_ENABLED false      // Set to "false" to disable the timer ISR (if getting "__vector_7" compile errors set this to false). Default=true
// For use with AT85 , Timer should be enabled
// ---------------------------------------------------------------------------
#include <NewPing.h> 
#include <util/delay.h>
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

#define TRIGGER_PIN PB0
#define ECHO_PIN PB1
#define DISTANCE 300

RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, DISTANCE); 
 
void setup() {
  driver.init();
}
void loop() {
  uint8_t distance = sonar.ping_cm();
//uint8_t distance = 1234;
  char msg[4];
  itoa(distance,msg,10);
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  _delay_ms(500);
}
