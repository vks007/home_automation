/*
 *myutils.h - Simple utilities commonly used in your programs
*/

#ifndef MYUTILS_H
#define MYUTILS_H


/*
 * This function provides a safe way to introduce delay by calling yield() every 10ms
 * Using this function will eliminate the chance of triggering a WDT
 * It seems this function may not be required in most cases, read here : https://www.sigmdel.ca/michel/program/esp8266/arduino/watchdogs_en.html
 */
void safedelay(unsigned int delay_time) {
  short n = delay_time / 10;//gives the no of times to loop with 10ms delay
  short remainder = delay_time % 10; //gives the remainder to give additional delay after looping n times
  for(int i = 0; i < n; i++) {
    delay(10);
    yield();
  }
  delay(remainder);
}

uint32_t RTCmillis() {
  // system_get_rtc_time() is in us (but very inaccurate anyway)
  return (system_get_rtc_time() * (system_rtc_clock_cali_proc() >> 12)) / 1000;
}

#endif
