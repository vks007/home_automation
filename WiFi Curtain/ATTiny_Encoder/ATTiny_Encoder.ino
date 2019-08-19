
/*
 * This is the most accurate encoder that I have till date. It counts 1 step for every 2 intends on the encoder. For mu nouse wheel it means 12 steps for a revolution
 * This encoder was originally written for encoder inputs at PB0/PB1 and consequently all operations are defined on bit0/bit1
 * I tried to modify this program to redefine the encoder inputs at PB3/PB4 but could not make it work.
 * So I decided to read values from PB3/PB4 and shift the values to bit0/bit1 and so the remaining program remains the same
 * Hurray! it works this way !
 */

#include <Wire.h>

#define ENC_RD  PINB  //encoder port read
#define ENC_A 3
#define ENC_B 4
#define INT_A PCINT3
#define INT_B PCINT4

volatile long counter = 0;
volatile long lastCounter = 0;

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
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  //This way of attaching interrupt does not work with ATTiny
  //attachInterrupt(PCINT0, evaluateRotary, CHANGE);
  //attachInterrupt(PCINT1, evaluateRotary, CHANGE);
  
  // Configure pin change interrupts on encoder pins
  PCMSK |= 1<<INT_A | 1<<INT_B;
  GIMSK = 1<<PCIE;                // Enable pin change interrupts
  GIFR = 1<<PCIF;                 // Clear pin change interrupt flag.

  //setup i2C on the SDA & SCL pins: By default on ATTiny85 has SDA on Pin 5 & SCL on Pin 7
  Wire.begin(8);                // join i2c bus with address #8 SCL on PB2 , SDA on PB0
  Wire.onRequest(requestEvent); // register event
}


/* encoder routine. Expects encoder with four state changes between detents
 * and both pins open on detent 
 *  This encoder code only measures 1 count on two steps of the encoder. 
 *  For the mouse encoder I use it takes 24 intends to complete a revolution and that is counted as 12 steps using this code
*/
ISR (PCINT0_vect) {
  static uint8_t old_AB = 3;  //lookup table index
  static int8_t encval = 0;   //encoder value  
  static const int8_t enc_states [] PROGMEM = 
  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table
  
  old_AB <<=2;  //remember previous state
  uint8_t enc = ENC_RD >>3; //shift bit5/4 to bit1/0 as I am working with the last 2 bits only (see at top for explaination)
  old_AB |= ( enc & 0x03 );
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
      i2c_regs[0] = counter >> 8;
      i2c_regs[1] = counter & 0xFF;
    lastCounter = counter;
  }

}


void loop() {

}
