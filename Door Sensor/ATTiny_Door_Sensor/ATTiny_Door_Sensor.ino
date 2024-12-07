/*
 * ****IMPORTANT************ => COMPILE PARAMETERS : Board: ATTiny13 , Processor: 13a , Processor Speed: 1.2 MHz , leave the rest to defaults as below:
 * Millis Avail, No Tone, LTO Enabled, Serial support: Half Duplex, Read+Write, BOD Level: 2.7v
 * 
 *Version 1.3 - Implements communication with ESP via 2 signal pins
 *This sketch is part of the sensor monitoring system. The ATTiny monitors the state of the sensor and outputs a short pulse on its output which is fed into the CH_PD of a ESP
 *Along with this it also outputs the signal type on 2 pins which are also decoded by the ESP as one of the three states - Wakeup , Sensor open, sensor close
 *Wakeup - The ATTiny wakes up at a certain interval to publish a "I am alive" message"
 *The ESP then processes this pulse and does the notification before shutting itself down
 *
 *Hardware connections for this circuit when using ESP01 module:
 *Vcc: 3.3 , GND , sensor magnetic switch between PB3 & PB4 , PB0 - o/p to ESP CH_PD , PB1/PB2 - output to Tx/Rx of ESP 
 *ATTiny13A   ESP8266-01
 *Vcc         Vcc
 *GND         GND
 *PB0         CH_PD
 *PB1         Tx //You can any GPIO pins on ESP for this
 *PB2         Rx //You can any GPIO pins on ESP for this
 *
 *Hardware connections for this circuit when using ESP12 module:
 *Vcc: 3.3 , GND , sensor magnetic switch between PB1 & PB2 , PB0 - o/p to ESP CH_PD , PB3/PB4 - output to GPIO5/GPIO4 of ESP 
 *ATTiny13A   ESP8266-12
 *Vcc         Vcc
 *GND         GND
 *PB0         CH_PD
 *PB3         GPIO5 //You can any GPIO pins on ESP for this
 *PB4         GPIO4 //You can any GPIO pins on ESP for this
 *
 *Concept to read state of the sensor: Here I use the concept of using 2 pins instead of 1 to read the value of a switch. By doing so I avoid the current flowing through the input PIN at all times. Here is what has been done:
 *If you can periodically check the state, using WDT rather than interrupts, then you can check with virtually no power.  Tie the reed switch between two GPIOs and then drive one pin to ground while 
 *the other pin is set to INPUT_PULLUP.  If the input pin follows the output then the switch is closed, if not then it's open. When done reading, drive the output high so there is no current through the switch
 *Current consumption analysis : When the sensor is closed, it takes 3 uA while when it is open it takes 700uA which is great!!!
 *MOSFET & HT7333 LDO takes 2uA current always
 *When idle : AT13 takes 7 uA , When active it takes 700 uA
 *One learning for me was that if I use the format of a main function and call setup() and put a infinite loop - then the current csonsumption of the AT is more as compared to
 *using the setup() and loop() construct. I dont know why is that so
 * Full instructable at : https://www.instructables.com/Wireless-Door-Sensor-Ultra-Low-Power/

ATTiny 13 pinout
                                    +---\/---+
     (Reset)              (D 5) PB5 |1      8| VCC
     (USB-) AnalogA3      (D 3) PB3 |2      7| PB2 (D 2) AnalogA1, I2C & SPI Clk
     (USB+) PWM, AnalogA2 (D 4) PB4 |3      6| PB1 (D 1) PWM1, SPI (LED via 470R)
                                GND |4      5| PB0 (D 0) PWM0, I2C, SPI
                                    +--------+

References:
Code credit : http://brownsofa.org/blog/2011/01/10/the-compleat-attiny13-led-flasher-part-3-low-power-mode/
See similar example of sleep using WDT here: https://arduinodiy.wordpress.com/2015/06/22/flashing-an-led-with-attiny13/
 */
 
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/power.h>
#include "ATTinyUART.h"

//#define F_CPU 1200000 // 1.2 MHz //Complile this with 1.2 MHz freq (will look into this later to fix the Clock freq by code)

#define ESP_12 //type of module for which this willbe compiled

// Utility macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))

// prescale WDT timer . See section Section 8.5.2 Table 8-2 in datasheet of t13A for values that can be set
// This value is the sleep time for the AT
//Set the time interval for WDT timer, WDT will trigger every N secs you configure here to check for the reed switch state
//If you keep this interval longer, state changes between the interval will be lost. I havent seen any difference in power consumption so i leave it at 0.5 sec
#define SLEEP_FOREVER  128
#define SLEEP_016MS    (period_t)0
#define SLEEP_125MS    (1<<WDP1) | (1<<WDP0)
#define SLEEP_250MS    (1<<WDP2)
#define SLEEP_500MS    (1<<WDP2) | (1<<WDP0)
#define SLEEP_1SEC     (1<<WDP2) | (1<<WDP1)
#define SLEEP_2SEC     (1<<WDP2) | (1<<WDP1) | (1<<WDP0)
#define SLEEP_4SEC     (1<<WDP3)
#define SLEEP_8SEC     (1<<WDP3) | (1<<WDP0)

//Types of messages sent to the signal pins
#define WAKEUP 1 // I am alive message
#define SENSOR_OPEN 2 //Sensor open message
#define SENSOR_CLOSE 3 //Sensor close message

#define SIGNAL_PULSE_LENGTH 5000 //pulse length in ms to be sent on sensor open event, //Complile this with 1.2 MHz freq
#define ENABLE_PULSE_LENGTH 2000//pulse length in ms to be sent on sensor open event , My ESP01 is working with 250mS but for ESP12 I think i need ~ 2000 ms, still trying to solve this
#define WAKEUP_COUNT 86400 //wake up interval count, this is a multiple of WDT timer prescaler. Eg. WAKEUP_COUNT* WDT_PRESCALER = Total time in sec, 
                        //Note: WDT precaling is independent of the clock speed. If WDT is set to 0.5 sec & WAKEUP_COUNT = 7200 , then Wake up time = 7200*0.5 = 3600 sec = 1 hr
//Example values for WAKEUP_COUNT with WDT set as 0.5 sec: 172800 = 24 hrs , 86400 = 12 hrs

//The pins which are connected between ESP and ATTiny are different according to the ESP Type, you may change this to suite your requirement
#if defined(ESP_12)
  #define SWITCH_INPUT1 PB1 //This is connected to the reed switch
  #define SWITCH_INPUT2 PB2 //This is connected to the reed switch
  #define ENABLE_PIN PB0 //This is connected to CH_PD on ESP
  #define SIGNAL_PIN0 PB3 //This is connected to GPIO pin on ESP as input. 
  #define SIGNAL_PIN1 PB4 //This is connected to GPIO pin on ESP as input. 
  //State Mapping of SIGNAL_PIN0 SIGNAL_PIN1:: 11=>IDLE , 00=> WAKEUP , 01=> SENSOR_OPEN , 10=> SENSOR_CLOSED
#elif defined(ESP01)
  #define SWITCH_INPUT1 PB3 //This is connected to the reed switch
  #define SWITCH_INPUT2 PB4 //This is connected to the reed switch
  #define ENABLE_PIN PB0 //This is connected to CH_PD on ESP
  #define SIGNAL_PIN0 PB1 //This is connected to GPIO pin on ESP as input. 
  #define SIGNAL_PIN1 PB2 //This is connected to GPIO pin on ESP as input. 
  //State Mapping of SIGNAL_PIN0 SIGNAL_PIN1:: 11=>IDLE , 00=> WAKEUP , 01=> SENSOR_OPEN , 10=> SENSOR_CLOSED
#else
  #error "ESP module type not defined correctly"
#endif

bool sensor_open = false; //indicates that the sensor is in open state
volatile bool wdt_event = false; // indicates that a WDT event has been fired
volatile long wakeup_counter = 0; //keeps a count of how many WDT events have been fired

//WDT fires at a predefined interval to sense the state of the sensor switch
ISR(WDT_vect) {
  //disable global interrupts
  cli();
  wakeup_counter++;
  wdt_event = true;
}

/*
 * sets the pin modes and enables the WDT timer
 */
void setup()
{
  //  if (F_CPU == 1200000) clock_prescale_set(clock_div_2);

  // Set up Ports direction
  //  DDRB = 0b00010111; This is how you do this in a single line but you have to hard code the ports
  pinMode(SWITCH_INPUT1, INPUT);
  pinMode(SWITCH_INPUT2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SIGNAL_PIN0, OUTPUT);
  pinMode(SIGNAL_PIN1, OUTPUT);
  
  bit_clear(PORTB, BIT(ENABLE_PIN)); //Write initial values of LOW on PB0
  bit_set(PORTB, BIT(SIGNAL_PIN0)); //Write initial values of HIGH on PB1
  bit_set(PORTB, BIT(SIGNAL_PIN1)); //Write initial values of HIGH on PB2
  bit_set(PORTB, BIT(SWITCH_INPUT1));//enable pull up on PB3
  
  WDTCR = SLEEP_500MS;
  
  // Enable watchdog timer interrupts
  WDTCR |= (1<<WDTIE);
  sei(); // Enable global interrupts 
  
}

/*
 * sendSingnal sends the signal to the ESP module on the two signal pins
 */
void sendSignal(byte mode)
{
  //Send a short HIGH to enable the ESP
  bit_set(PORTB, BIT(ENABLE_PIN));//send a high pulse on PB0 to wake up the ESP
  _delay_ms(ENABLE_PULSE_LENGTH);
  bit_clear(PORTB, BIT(ENABLE_PIN));//finish the pulse
  _delay_ms(1);

  //Send the signal according to the mode
  if(mode == WAKEUP)//send 00
  {
    bit_clear(PORTB, BIT(SIGNAL_PIN0));
    bit_clear(PORTB, BIT(SIGNAL_PIN1));
  }
  else if(mode == SENSOR_OPEN)//send 01
  {
    bit_clear(PORTB, BIT(SIGNAL_PIN0));
    bit_set(PORTB, BIT(SIGNAL_PIN1));
  }
  else if(mode == SENSOR_CLOSE)//send 10
  {
    bit_set(PORTB, BIT(SIGNAL_PIN0));
    bit_clear(PORTB, BIT(SIGNAL_PIN1));
  }
  _delay_ms(SIGNAL_PULSE_LENGTH);//maintain this signal for a certain time
  //Revert the signal to IDLE state which is 11
  bit_set(PORTB, BIT(SIGNAL_PIN0));
  bit_set(PORTB, BIT(SIGNAL_PIN1));
}

/* 
 *  set system into the sleep state
 * system wakes up when watchdog times out
*/
void system_sleep() {
  
  ACSR = ADMUX = ADCSRA = 0;  
  ACSR |= (1 << ACD);                  //Analog comparator off
  ADCSRA &= ~(1<<ADEN);                // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  power_all_disable();
  sleep_enable();
  cli();
  //Beaware that if you disable BOD then if the voltage level falls below the critical level then the chip
  // will continue to operate unpredictably. This however saves idle current (~ 7uA)
  BODCR = (1<<BODSE)|(1<<BODS);       //Disable BOD, step 1
  BODCR = (1<<BODS);                  //Second step
  sei();                              //Enable global interrupts 
  sleep_cpu();  //Do not use sleep_mode(), contains sleep_enable(), sleep_cpu(),sleep_disable()
  sleep_disable();
  power_all_enable();
}

void loop() {
    if(wdt_event)
    {
      wdt_event = false;
      //write a LOW on PB4 to read PB3
      // PORTB &= 0b11101111; //write a LOW on PB4 to read PB3
      bit_clear(PORTB, BIT(SWITCH_INPUT2));
      //give a delay here as the MCU needs some time to stablize the ouput in previous step. Else the MCU may read wrong value
      _delay_us(1);
      //if((PINB & 0x08))//If PB3 is 1, sensor switch is open  // TO DO : Remove hard coding of Pins in this statement
      if(bit_get(PINB,BIT(SWITCH_INPUT1)))
      {
        //uart_puts("B3:1\n");
        if(!sensor_open) //send a HIGH pulse on Pb0 only if the sensor was reviously closed
        {
          sensor_open = true; //remember the state of the sensor
          sendSignal(SENSOR_OPEN);
        }
      }
      else //sensor switch is in closed position
      {
        if(sensor_open)//send a pulse only if the sensor was open previously
        {
          sensor_open = false;
          sendSignal(SENSOR_CLOSE);
        }
      }

      //We had set SWITCH_INPUT2 LOW only to read the input, SWITCH_INPUT1 , now that we're done set it to HIGH again
      // PORTB |= 1<<PB4;//Write a HIGH on PB4
      bit_set(PORTB, BIT(SWITCH_INPUT2));

      //check if its time to wake up anyway irrespective of the sensor position
      if(wakeup_counter >= WAKEUP_COUNT)
      {
        wakeup_counter = 0;//reset the counter
        sendSignal(WAKEUP);
      }
      
    } //end if(wdt_event)
    else
      system_sleep();
}
