
/*
 * This is the most accurate encoder that I have till date. Source : https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/
 */

#include "ATTinyUART.h"

#define ENC_RD  PINB  //encoder port read
#define ENC_A 0 
#define ENC_B 1 
#define        UART_BAUDRATE   115200

volatile long counter = 0;
volatile long lastCounter = 0;

void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  //This way of atatching interrupt does not work with ATTiny
  //attachInterrupt(PCINT0, evaluateRotary, CHANGE);
  //attachInterrupt(PCINT1, evaluateRotary, CHANGE);
  
  // Configure pin change interrupts on PB1, PB2, and PB3
  PCMSK |= 1<<PCINT0 | 1<<PCINT1;
  GIMSK = 1<<PCIE;                // Enable pin change interrupts
  GIFR = 1<<PCIF;                 // Clear pin change interrupt flag.

//  uart_puts("Starting!\n");
}


/* encoder routine. Expects encoder with four state changes between detents */
/* and both pins open on detent */
ISR (PCINT0_vect) {
  static uint8_t old_AB = 3;  //lookup table index
  static int8_t encval = 0;   //encoder value  
  static const int8_t enc_states [] PROGMEM = 
  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table
  
  old_AB <<=2;  //remember previous state
  old_AB |= ( ENC_RD & 0x03 );
  encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
  
  if( encval > 1 ) {  //two steps forward
    counter++;
    encval = 0;
  }
  else if( encval < -1 ) {  //two steps backwards
    counter--;
    encval = 0;
  }

//Prints the value from the encoder when it changes to the Tx pin. This can read off the pin via a SoftwareSerial interface
  if(counter != lastCounter){
    uart_putl(counter);
    uart_putc('\n');
    lastCounter = counter;
  }

}


void loop() {

}


