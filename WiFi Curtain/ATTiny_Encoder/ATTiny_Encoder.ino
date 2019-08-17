
/*
 * This is the most accurate encoder that I have till date. Source : https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/
 */

#include <Wire.h>

#define ENC_RD  PINB  //encoder port read
#define ENC_A PB3
#define ENC_B PB4

volatile int16_t counter = 0;
volatile int16_t lastCounter = 0;

volatile uint8_t i2c_regs[] =
{
    0, //older 8
    0 //younger 8
};

volatile byte reg_position = 0;
const byte reg_size = sizeof(i2c_regs);

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
// This function transmits the encoder value as characters as 2 bytes. It can send a max value of 65535
void requestEvent() {
  //Write 2 bytes of data => higher byte and lower byte
  while(reg_position < reg_size) {
    Wire.write(i2c_regs[reg_position]); // respond with message of 6 bytes
    reg_position++;
  }
  if (reg_position >= reg_size) //reset the counter to 0 for next request event
      reg_position = 0;
}

void setup() {
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  //This way of attaching interrupt DOES NOT work with ATTiny
  //attachInterrupt(PCINT0, evaluateRotary, CHANGE);
  //attachInterrupt(PCINT1, evaluateRotary, CHANGE);
  
  // Configure pin change interrupts on PB1, PB2, and PB3
  PCMSK |= 1<<PCINT3 | 1<<PCINT4;
  GIMSK = 1<<PCIE;                // Enable pin change interrupts
  GIFR = 1<<PCIF;                 // Clear pin change interrupt flag.

  //setup i2C on the SDA & SCL pins: By default on ATTiny85 has SDA on Pin 5 & SCL on Pin 7
  Wire.begin(8);                // join i2c bus with address #8 SCL on PB2 , SDA on PB0
  Wire.onRequest(requestEvent); // register event
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

  //store the value of the counter in the i2c array
  i2c_regs[0] = counter >> 8;
  i2c_regs[1] = counter & 0xFF;
  lastCounter = counter;

}


void loop() {

}
