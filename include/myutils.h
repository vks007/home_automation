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


/*
 * Converts an IPAddress type to a string
 */
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}


template <class X, class M, class N, class O, class Q>
X map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*
 * Converts milli seconds into a readable short string in the format days hour:min:sec ago. example : 5 days 04:09:16 ago
 */
String getReadableTime(unsigned long millis) {
  String readableTime = "";
  unsigned long seconds;
  unsigned long minutes;
  unsigned long hours;
  unsigned long days;

  seconds = millis / 1000;
  minutes = seconds / 60;
  hours = minutes / 60;
  days = hours / 24;
  millis %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  readableTime = String(days) + " ";
  
  if (hours < 10) {
    readableTime += "0";
  }
  readableTime += String(hours) + ":";

  if (minutes < 10) {
    readableTime += "0";
  }
  readableTime += String(minutes) + ":";

  if (seconds < 10) {
    readableTime += "0";
  }
  readableTime += String(seconds) + " ago";
  return readableTime;
}

#if defined(ESP8266)
uint32_t RTCmillis() {
  // system_get_rtc_time() is in us (but very inaccurate anyway)
  return (system_get_rtc_time() * (system_rtc_clock_cali_proc() >> 12)) / 1000;
}

// Write 16 bit int to RTC memory with checksum, return true if verified OK
// Slot 0-127
// (C) Turo Heikkinen 2019 , Source: https://hackaday.io/project/24993/instructions
bool writeRtcMem(uint16_t *inVal, uint8_t slot = 0) {
  uint32_t valToStore = *inVal | ((*inVal ^ 0xffff) << 16); //Add checksum
  uint32_t valFromMemory;
  if (ESP.rtcUserMemoryWrite(slot, &valToStore, sizeof(valToStore)) &&
      ESP.rtcUserMemoryRead(slot, &valFromMemory, sizeof(valFromMemory)) &&
      valToStore == valFromMemory) {
        return true;
  }
  return false;
}

// Read 16 bit int from RTC memory, return true if checksum OK
// Only overwrite variable, if checksum OK
// Slot 0-127
// (C) Turo Heikkinen 2019 , Source: https://hackaday.io/project/24993/instructions
bool readRtcMem(uint16_t *inVal, uint8_t slot = 0) {
  uint32_t valFromMemory;
  if (ESP.rtcUserMemoryRead(slot, &valFromMemory, sizeof(valFromMemory)) &&
      ((valFromMemory >> 16) ^ (valFromMemory & 0xffff)) == 0xffff) {
        *inVal = valFromMemory;
        return true;
  }
  return false;
}
#endif

/*
This function is a fast and memory efficient way to compare two strings
Function returns true on success (strings are the same), else false
Source/Credit: https://blog.podkalicki.com/fast-string-comparison-for-microcontrollers/
*/
static bool xstrcmp(const char *s1, const char *s2)
{
    while(*s1 != '\0' && *s1 == *s2) {s1++; s2++;}
    return((*s1 - *s2) == 0);
}

/*
This function validates a MAC string 
Function returns true if input string is a valid MAC else false
Source/Credits: https://github.com/aardsoft/MACTool/blob/master/MACTool.cpp
*/
bool validate_MAC(const char *mac_string){
  char mac_address[18];
  char *t_ptr;

  strcpy(mac_address, mac_string);
  t_ptr = strtok(mac_address, ":");

  int i = 0;
  while (t_ptr != NULL){
    int octet = strtol(t_ptr, NULL, 16);
    if (octet < 0 || octet > 255)
      return false;

    t_ptr = strtok(NULL, ":");
    i++;
  }

  // loop should've covered exactly 6 octets
  if (i != 6)
    return false;

  return true;
}

/*
This function converts a MAC to a String 
Function returns a String
Source/Credits: https://github.com/aZholtikov/ZHNetwork/blob/main/src/ZHNetwork.cpp
*/
String macToString(const uint8_t *mac)
{
    String string;
    const char baseChars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    for (uint32_t i{0}; i < 6; ++i)
    {
        string += (char)pgm_read_byte(baseChars + (mac[i] >> 4));
        string += (char)pgm_read_byte(baseChars + mac[i] % 16);
    }
    return string;
}

/*
This function converts a MAC in String format to a MAC (uint8_t* array)
Function returns uint8_t array
Source/Credits: https://github.com/aZholtikov/ZHNetwork/blob/main/src/ZHNetwork.cpp
*/
uint8_t* stringToMac(const String &string, uint8_t *mac)
{
    const uint8_t baseChars[75]{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0,
                                10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 0, 0, 0, 0, 0, 0,
                                10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
    for (uint32_t i = 0; i < 6; ++i)
        mac[i] = (pgm_read_byte(baseChars + string.charAt(i * 2) - '0') << 4) + pgm_read_byte(baseChars + string.charAt(i * 2 + 1) - '0');
    return mac;
}


/*
This function converts a hex char to number
Returns: -1 if successful else the number equivalent
Source/Credits: https://android.googlesource.com/platform/external/wpa_supplicant_8/+/ics-mr1/src/utils/common.c
*/
static int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

/*
This function converts a hex char to byte , see stringToMac above for another variant
Returns: -1 if successful else the byte equivalent
Source/Credits: https://android.googlesource.com/platform/external/wpa_supplicant_8/+/ics-mr1/src/utils/common.c
*/
int hex2byte(const char *hex)
{
	int a, b;
	a = hex2num(*hex++);
	if (a < 0)
		return -1;
	b = hex2num(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}


/*
This function converts a MAC string to an uint8_t array
Returns: true is successful else false
Source/Credits: https://android.googlesource.com/platform/external/wpa_supplicant_8/+/ics-mr1/src/utils/common.c
*/
bool MAC_to_array(const char *hex, uint8_t *buf, size_t len)
{
	size_t i;
	int a;
	const char *ipos = hex;
	uint8_t *opos = buf;
	for (i = 0; i < len; i++) {
		a = hex2byte(ipos);
		if (a < 0)
			return false;
		*opos++ = a;
		ipos += 2;
	}
	return true;
}

//Checks if a string is positive numeric - only 0-9
bool isNumeric(String str)
{
  for(unsigned short i=0;i<str.length();i++)
    if(!isDigit(str.charAt(i)))
      return false;
  return true;
}

//Checks if a string is positive or negative numeric -> 0-9 along with negative sign
bool isSignedNumeric(String str)
{
  char ch='\0';
  for(unsigned short i=0;i<str.length();i++)
  {
    ch = str.charAt(i);
    if(ch== 45 && i != 0)//If we have a minus sign but it is not at the first position then it is not a valid numeric
      return false;
    //0-9 is ascii value 48-57 , minus sign is ascii value 45
    if(( ch >= 48 && ch <= 57) || ch == 45)
      continue;
    else
      {return false;}
  }
  return true;

/*
  if(str.length()> 0 && str.charAt(0) == '-')
    str.replace('-','');
  DPRINTLN("str.length2=" + String(str.length()));
  for(unsigned short i=0;i<str.length();i++)
    if(!isDigit(str.charAt(i)))
      {DPRINT("false");
      return false;}
    else
      {DPRINT("true");}
*/
}

//Formats time passed in seconds into hour:minute:second format and returns as string
String formatTime(unsigned long sec)
{
  unsigned int hr=0,mn=0;
  if(sec >= 60)
  {
    mn = sec/60;
    sec = sec - mn*60;
    if(mn >= 60)
    {
      hr = mn/60;
      mn = mn - hr*60;
    }
  }
  return String(hr) + ":" + String(mn) + ":" + String(sec);
}

#endif



