/*
 * ****IMPORTANT************ => COMPILE PARAMETERS : Board: ATTiny13 , Processor: 13a , Processor Speed: 1.2 MHz , leave the rest to defaults as below:
 * Millis Avail, No Tone, LTO Enabled, Serial support: Half Duplex, Read+Write, BOD Level: 2.7v
 * Library used : DIY ATtiny from here: https://github.com/sleemanj/optiboot/blob/master/dists/README.md
 * 
 *Version 1.4 - Implements communication with ESP via a single signal pin
 *This sketch is part of the low power sensor using a ATTiny and ESP. 
 *This sketch monitors the state of the sensor contact and triggers a wakeup of the ESP when it changes. To conserve battery it sleeps most of the times, waking up every N secs
 *via a WDT reset. The WDT event sets a bool variable which then triggers the rest of the flow in the main loop
 *The ATTiny monitors the state of the sensor and outputs a short pulse on its output which is fed into the CH_PD of a ESP
 *Along with this it also outputs the signal type on a pin which are also decoded by the ESP as one of the two states - Sensor open, Sensor close
 *Wakeup - The ATTiny wakes up at a certain interval to publish the state of the door, this message is no different than a open/close message , its just that the wake up message is a way to ensure
 *that the sensor is working fine. 
 *The ESP then processes this pulse and does the notification before shutting itself down , The ESP uses espnow to send a message to a gateway but this has no connection to the ATTiny
 *
 *ATtiny connections with ESP01:
 *ATTiny13A
 *PB3         Sensor contact 1
 *PB4         Sensor contact 2
 *PB1         NC
 *
 *Hardware connections between ATTiny and ESP for this circuit when using ESP01 module:
 *Vcc: 3.3 , GND , sensor magnetic switch between PB3 & PB4 , PB0 - o/p to ESP CH_PD , PB1/PB2 - output to Tx/Rx of ESP 
 *ATTiny13A   ESP8266-01
 *Vcc         Vcc (3V3)
 *GND         GND
 *PB0         CH_PD
 *PB2         Rx //Using Rx has the advantage of a visual led indication when the sensor is in open state
 *
 *Hardware connections between ATTiny and ESP for this circuit when using ESP12 module:
 *Vcc: 3.3 , GND , sensor magnetic switch between PB1 & PB2 , PB0 - o/p to ESP CH_PD , PB3/PB4 - output to GPIO5/GPIO4 of ESP 
 *ATTiny13A   ESP8266-12
 *Vcc         Vcc
 *GND         GND
 *PB0         CH_PD
 *PB4         GPIO5 //You can any GPIO pins on ESP for this
 *
 *ATtiny connections with ESP12:
 *ATTiny13A
 *PB0         Sensor contact 1
 *PB1         Sensor contact 2
 *PB3         NC
 *Concept to read state of the sensor: Here I use the concept of using 2 pins instead of 1 to read the value of a switch. By doing so I avoid the current flowing through the input PIN at all times. Here is what has been done:
 *
 *If you can periodically check the state, using WDT rather than interrupts, then you can check with virtually no power.  Tie the reed switch between two GPIOs and then drive one pin to ground while 
 *the other pin is set to INPUT_PULLUP.  If the input pin follows the output then the switch is closed, if not then it's open. When done reading, drive the output high so there is no current through the switch
 *Current consumption analysis : When the sensor is closed, it takes 3 uA while when it is open it takes 700uA which is great!!!
 *MOSFET & HT7333 LDO takes 2uA current always
 *When idle : AT13 takes 7 uA , When active it takes 700 uA
 *On idle the entire circuit takes ~28-30uA , in spite of the fact that the ESP should take zero current when CH_PD is LOW, I think other components end up making up some idle current. I havent investigated this yet
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

//#define F_CPU 1200000 // 1.2 MHz //Complile this with 1.2 MHz freq (will look into this later to fix the Clock freq by code)

#define ESP01 //type of module for which this willbe compiled . ESP01 or ESP12

// Utility macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))

//Types of messages sent to the signal pins
#define SENSOR_OPEN 1 //Sensor open message
#define SENSOR_CLOSE 2 //Sensor close message

#define SIGNAL_PULSE_LENGTH 200 //pulse length in ms to be sent on sensor open event, //Complile this with 1.2 MHz freq
#define ENABLE_PULSE_LENGTH 250//pulse length in ms to be sent on sensor open event
#define WAKEUP_COUNT 86400 //wake up interval count, this is a multiple of WDT timer prescaler. Eg. WAKEUP_COUNT* WDT_PRESCALER = Total time in sec, 
                        //Note: WDT precaling is independent of the clock speed. If WDT is set to 0.5 sec & WAKEUP_COUNT = 7200 , then Wake up time = 7200*0.5 = 3600 sec = 1 hr
//Example values for WAKEUP_COUNT with WDT set as 0.5 sec: 172800 = 24 hrs , 86400 = 12 hrs

//The pins which are connected between ESP and ATTiny are different according to the ESP Type, you may change this to suite your requirement
#if defined(ESP_12)
  #define SWITCH_INPUT1 PB1 //This is connected to the reed switch
  #define SWITCH_INPUT2 PB2 //This is connected to the reed switch
  #define ENABLE_PIN PB0 //This is connected to CH_PD on ESP
  #define SIGNAL_PIN PB4 //This is connected to GPIO pin on ESP as input. 
#elif defined(ESP01)
  #define SWITCH_INPUT1 PB3 //This is connected to the reed switch
  #define SWITCH_INPUT2 PB4 //This is connected to the reed switch
  #define ENABLE_PIN PB0 //This is connected to CH_PD on ESP
  #define SIGNAL_PIN PB2 //This is connected to GPIO pin on ESP as input. 
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
  pinMode(SWITCH_INPUT1, INPUT_PULLUP);
  pinMode(SWITCH_INPUT2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SIGNAL_PIN, OUTPUT);
  
  bit_clear(PORTB, BIT(ENABLE_PIN)); //Write initial values of LOW on ENABLE_PIN
  //bit_set(PORTB, BIT(SIGNAL_PIN0)); //Write initial values of HIGH on SIGNAL PIN
  bit_set(PORTB, BIT(SIGNAL_PIN)); //Write initial values of HIGH on SIGNAL PIN
  bit_set(PORTB, BIT(SWITCH_INPUT1));//enable pull up on PB3
  
  // prescale WDT timer . See section Section 8.5.2 Table 8-2 in datasheet of t13A
  //Set the time interval for WDT timer, WDT will trigger every N secs you configure here to check for the reed switch state
  //If you keep this interval longer, state changes between the interval will be lost. I havent seen any difference in power consumption so i leave it at 0.5 sec
  //WDTCR = (1<<WDP2);// 0.25 sec
  WDTCR = ((1<<WDP2) | (1<<WDP0));// 0.5 sec
  //WDTCR = ((1<<WDP2) | (1<<WDP1));// 1 sec
  //WDTCR = ((1<<WDP2) | (1<<WDP1) | (1<<WDP0));// 2 sec
  //WDTCR = (1<<WDP3) ;// 4 sec
  
  // Enable watchdog timer interrupts
  WDTCR |= (1<<WDTIE);
  sei(); // Enable global interrupts 
  
}

/*
 * sendSingnal sends the signal to the ESP module on the two signal pins
 */
void sendSignal(byte mode)
{
  // !!!! IMPORTANT !!!!!!
  // First send the enable pulse else the logic conditions of the Signal pins can affect the boot mode . I spend many days debugging why ESP was not starting up
  // It was because I was sending the signal first which held either Rx or Tx LOW , and the ESP wont start
  
  //Send a short HIGH to enable the ESP
  bit_set(PORTB, BIT(ENABLE_PIN));//send a high pulse on PB0 to wake up the ESP
  _delay_ms(ENABLE_PULSE_LENGTH);
  bit_clear(PORTB, BIT(ENABLE_PIN));//finish the pulse
  _delay_ms(1);

  if(mode == SENSOR_OPEN)//send 1
  {
    bit_set(PORTB, BIT(SIGNAL_PIN));
  }
  else if(mode == SENSOR_CLOSE)//send 0
  {
    bit_clear(PORTB, BIT(SIGNAL_PIN));
  }
  
  _delay_ms(SIGNAL_PULSE_LENGTH);//maintain this signal for a certain time
  // There is no need to revert the signal to a particular state, I dont see any current consumption difference on either state. havent checked if it anyway reverts to some state during sleep

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
      //write a LOW on SWITCH_INPUT2 to read SWITCH_INPUT1
      bit_clear(PORTB, BIT(SWITCH_INPUT2));
      //give a delay here as the MCU needs some time to stablize the ouput in previous step. Else the MCU may read wrong value
      _delay_us(1);
      // According to my logic if the bit is set then the sensor is open but it is behaving in an opposite way
      // havent been able to figure this out so I am just inverting the condition in the if statement below, will worry about it later
      if(!bit_get(PINB,BIT(SWITCH_INPUT1)))
      {
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
        //send the current state of the sensor
        if(sensor_open)
          sendSignal(SENSOR_OPEN);
        else
          sendSignal(SENSOR_CLOSE);
      }
      
    } //end if(wdt_event)
    else
      system_sleep();
}
