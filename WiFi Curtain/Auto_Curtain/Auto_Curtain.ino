/*
 * Version history 
 * 1.1 - change the logic of trigger to that of continous movement for the curtain when the button is pressed
 * 1.2 - Included OTA and included all javascript to support both the touch based mobile browser and mouse based computer browser
 * 1.3 - modifying program to send signals for renewal of ON time rather than wait for the stop signal. Changed Hostname
 * 1.4 - modifyied to include stopping of curtain based on encoder inputs
 * 
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <ArduinoOTA.h>
#include <Automaton.h>
#include <FS.h>
#include <SoftwareSerial.h>

#define DEBUG
#define VERSION 1.52
#define HOSTNAME "MBRCurtain_" //< Hostname. The setup function adds the Chip ID at the end.
#define ENCODER_WAIT_TIME 3000 //time in ms
#define DEBOUNCE_INTERVAL 300 //time in ms
#define INIT_ENCODER_VALUE 9999 //Initial value to detect that we havent read any values from the encoder yet

const char *ssid = "EAGLE"; //SSID of WiFi to connect to
const char *password = "HELLO99020"; //Password of WiFi to connect to

bool servoRunning = false;
bool curtainDirectionUp = true;
volatile bool btnUp = false;//push button to trigger a curtain to move up
volatile bool btnDn = false;//push button to trigger a curtain to move down
volatile bool btnTop = false;//monitors if the curtain has reached the extreme top position, normally this should never be the case. monitored via a hall sensor
unsigned long lastDebounceTimeUp = 0;
unsigned long lastDebounceTimeDn = 0;
unsigned long lastEncoderTime = 0;//stores the last time we detected a change in the encoder reading

//PIN Definitions
short MOTOR_PIN = D5;//pin on which motor for curtain is attached
short MOVE_UP_PIN = D1;//input pin to move curtain up
short MOVE_DN_PIN = D2;//input pin to move curtain down
short ENCODER_RX_PIN = D6;//ESP receives the encoder positions from the ATiny on this pin and can be read off via a Software Serial
short ENCODER_TX_PIN = D7;//ESP receives the encoder positions from the ATiny on this pin and can be read off via a Software Serial
short CURTAIN_TOP_PIN = D4;//Pin to sense if the curtain has reached at its top end - a signal for emegency stop and reset the curtain as something seems to have gone wrong.

int encoderPosition = INIT_ENCODER_VALUE;
int lastEncoderPosition = INIT_ENCODER_VALUE;
int curtainMaxSteps = 200; // maximum no of steps that the curtain can move before it stops.This value is actually picked form the config file on the ESP, not here, but just in case
int currentStepPosition = 0;//maintains the no of steps that the servo has moved , One full round consists of 23 steps by the encoder I am using.
bool crashRecovery = false;//indicates if ESP has recovered from a power off/crash while the curtain was in motion
int tmp_interval = 0;

struct state{
  short steps;
  bool isRunning;
  short maxSteps;
};

state last_state; //stores the last state that was saved to the file

Servo myservo;
std::unique_ptr<ESP8266WebServer> httpServer; //Ref : https://gist.github.com/tzapu/ecc0759829d30d5a6152
SoftwareSerial ESPserial(ENCODER_RX_PIN, ENCODER_TX_PIN); // RX | TX

const char rootHTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset=utf-8 />
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>Automated Curtain</title>
  <link rel='stylesheet' type='text/css' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.4/css/bootstrap.min.css'>
  <script type='text/javascript' src='https://code.jquery.com/jquery-2.1.4.min.js'></script>
  <script type='text/javascript' src='https://cdn.rawgit.com/Foliotek/AjaxQ/master/ajaxq.js'></script>

  <script>

window.onload = function() {
    $("#btn_up").bind('click', curtainUp)
    $("#btn_down").bind('click', curtainDn)
    $("#btn_stop").bind('click', curtainStop)
    $("#btn_reset").bind('click', curtainReset)
/*
    $("#btn_up").bind('mousedown', curtainUp)
    .bind('mouseup', curtainStop);

    $("#btn_down").bind('mousedown', curtainDn)
    .bind('mouseup', curtainStop);
*/
}

function curtainUp()
{
  $.get( '/up')  ;
  {Connection: close};
}
function curtainDn()
{
  $.get( '/dn')  ;
  {Connection: close};
}

function curtainStop()
{
  $.get( '/st')  ;
  {Connection: close};
}
function curtainReset()
{
  if(confirm("The curtain has to be at the bottom before reset. Are you sure?")) {
    $.get( '/rst')  ;
    {Connection: close};
  }
}
</script>
</head>
<body>
<div class='container'>
<h1>Automated Curtain</h1>
<div class='row'>
  <div class='col-md-5'></div>
  <div class='col-md-2'>
    <button id='btn_up' class='btn btn-primary btn-block' type='button'>Curtain Up</button>
  </div>
  <div class='col-md-5'></div>
</div>
</br></br></br>
<div class='row'>
  <div class='col-md-5'></div>
  <div class='col-md-2'>
    <button id='btn_down' class='btn btn-primary btn-block' type='button' >Curtain Down</button>
  </div>
  <div class='col-md-5'></div>
</div>
</br></br></br>
<div class='row'>
  <div class='col-md-5'></div>
  <div class='col-md-2'>
    <button id='btn_stop' class='btn btn-primary btn-block' type='button'>Stop Curtain</button>
  </div>
  <div class='col-md-5'></div>
</div>
</br></br></br>
<div class='row'>
  <div class='col-md-5'></div>
  <div class='col-md-2'>
    <button id='btn_reset' class='btn btn-primary btn-block' type='button'>Reset Curtain</button>
  </div>
  <div class='col-md-5'></div>
</div>
</div>
</body>
</html>
)=====";


void setup() {
  #if defined(DEBUG)
    Serial.begin(115200);
    delay(1);
  #endif

  //This Serial is to get the encoder data from the AVR, encoder measurement is done by a AVR , not ESP
  ESPserial.begin(9600);
  debugln("");

  pinMode(MOVE_UP_PIN,INPUT);
  pinMode(MOVE_DN_PIN,INPUT);
  pinMode(ENCODER_RX_PIN,INPUT);
  pinMode(ENCODER_TX_PIN,INPUT);//This pin is not used but required for ESPSerial
  pinMode(MOTOR_PIN,OUTPUT);
  pinMode(CURTAIN_TOP_PIN,INPUT);

  WiFiSetup();

  //Call OTA setup code
  OTASetup();
  
  if(WiFi.status() == WL_CONNECTED)
  {
    //Handle routines for server URLs
    httpServer.reset(new ESP8266WebServer(WiFi.localIP(), 80));
    httpServer->on ( "/", handleRoot );
    httpServer->on ( "/up", handleCurtainUp );
    httpServer->on ( "/dn", handleCurtainDn );
    httpServer->on ( "/st", handleServoStop );

    httpServer->on ("/version",handleVersion);
    httpServer->on ("/rst",handleReset);
    httpServer->on ("/csh",handleCrash);
    httpServer->on ("/dbg",handleDebug);
    httpServer->on ("/sav",handleSaveState);
    httpServer->begin();

    debugln ( "HTTP server started" );
  }
  
  //Initialize the encoder current position by reading the last values stored in the config file
  state init_state = getCurrentState();
  currentStepPosition = init_state.steps;
  curtainMaxSteps = init_state.maxSteps;
  debugln("Current steps:" + String(currentStepPosition) + ", maxSteps:" + String(curtainMaxSteps) + " running status:" + String(init_state.isRunning));
  debug("Initial enPos:");debug(String(encoderPosition));debug(" lstPos:");debugln(String(lastEncoderPosition));

  //This indicates the ESP crashed/turned off while it was running and hence was not able to store the last step state. This condition is non-recoverable and needs to be flagged so that a reset can be performed
  if(init_state.isRunning)
  {
    debugln("Entering crash recovery mode....");
    crashRecovery = true;
  }
  //crashRecovery = true;
  
  //Attach the interrupts on input pins as the last step
  attachInterrupt(MOVE_UP_PIN, ISRCurtainUp, FALLING);
  attachInterrupt(MOVE_DN_PIN, ISRCurtainDn, FALLING);
  attachInterrupt(CURTAIN_TOP_PIN, ISRCurtainTop, LOW);
}

void WiFiSetup()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin ( ssid, password );
  
  // Wait for connection to the WiFi
  for(short i=0;i<60;i++) {
    if(WiFi.status() != WL_CONNECTED)
    {
      delay ( 500 );
        debug( "." );
    }
    else
      break;
    yield();//Feed the watch dog timer
    }

  if(WiFi.status() == WL_CONNECTED)
  {    debugln ( "" );debug( "Connected to " );debugln ( ssid );debug( "IP address: " );
    debugln( WiFi.localIP().toString());//assign a static IP for this ESP's MAC address in the router for faster connection
  }

}
void OTASetup()
{
  debugln("\r\n");
  debug("Chip ID: 0x");
  debugln(String(ESP.getChipId(), HEX));
  
  // Set Hostname.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);

  // Print hostname.
  debugln("Hostname: " + hostname);
  //debugln(WiFi.hostname());

  // Start OTA server
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.onStart([]() { // switch off all outputs while upgrading but dont store this interim state on EPROM
                    
                    });

  ArduinoOTA.onEnd([]() { // do a fancy thing with our board led at end
                            //nothing to do here, yet
                        });

   ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });

  ArduinoOTA.begin();
  debugln("OTA Server setup successfully");
}

void loop() {
  // Handle OTA server
  ArduinoOTA.handle();
  yield();

  httpServer->handleClient();
  yield();

  if(btnTop == true && servoRunning && curtainDirectionUp)
  {
    curtainStop();
    debugln("curtain unexpectedly reached the extreme top and hence stopped.");
  }
  
  if(servoRunning)
  {
    updateEncoder();
    if(encoderPosition != lastEncoderPosition)//encoder has moved, servo is running
    {
      lastEncoderTime = millis();//update the timestamp as we've received a change in encoder value
      debug("enPos:");debug(String(encoderPosition));debug(" lstPos:");debug(String(lastEncoderPosition));debug(" stepPos:");debug(String(currentStepPosition));
      //Check if this is the first movement of encoder we have received after startup. if yes then special handling of lastEncoderPosition will have to be done
      if(lastEncoderPosition == INIT_ENCODER_VALUE)
      {
        if(curtainDirectionUp)
          lastEncoderPosition = encoderPosition +1;//assume we are moving from 1 notch before (counting decreases when moving up)
        else
          lastEncoderPosition = encoderPosition -1;//assume we are moving from 1 notch after (counting increases when moving down)
      }
      
      tmp_interval = (encoderPosition - lastEncoderPosition);
      debugln("interval:" + String(tmp_interval));
      currentStepPosition +=  tmp_interval;
      lastEncoderPosition = encoderPosition;

      debug(" new stepPos:");debugln(String(currentStepPosition));
      //Steps start from max position which is the bottom end of the curtain. Moving up decreases the steps , moving down increases the steps
      if(((currentStepPosition >= curtainMaxSteps) || (currentStepPosition <= 0)) && !crashRecovery)//we've reached the bottom or we've reached the top. Also dont stop automatically if we are in crash recovery mode  
        curtainStop();
    }
    else if((millis() - lastEncoderTime) > ENCODER_WAIT_TIME)
    {
      curtainStop();
      debugln("Curtain stopped as no encoder change received for more than " + String(ENCODER_WAIT_TIME) + " ms");
    }
    
  }


  if(btnUp == true)
  {
//    debugln("Up button pressed");
    btnUp = false;
    if(digitalRead(MOVE_UP_PIN) == LOW)//button is pressed
    {
      if(millis()-lastDebounceTimeUp > DEBOUNCE_INTERVAL )
      {
        lastDebounceTimeUp = millis();
        //Button works as a toggle switch turing it ON/OFF on successive press
        if(servoRunning)
          curtainStop();
        else
          curtainUp();
      }//else ignore the command and wait for this time be elapsed, the loop will execute again and again till it crosses DEBOUNCE_INTERVAL
    }
  }
  
  if(btnDn == true)
  {
//    debugln("Down button pressed");
    btnDn = false;
    if(digitalRead(MOVE_DN_PIN) == LOW)//button is pressed
    {
      if(millis()-lastDebounceTimeDn > DEBOUNCE_INTERVAL )
      {
        lastDebounceTimeDn = millis();
        //Button works as a toggle switch turing it ON/OFF on successive press
        if(servoRunning)
          curtainStop();
        else
          curtainDn();
      }//else ignore the command and wait for this time be elapsed, the loop will execute again and again till it crosses DEBOUNCE_INTERVAL
    }
  }

 
}


void updateEncoder()
{
  static String tmp = "";
  static short sign = 1;
  tmp = "";
  sign = 1;
  if ( ESPserial.available() )   
  {  
    tmp = ESPserial.readStringUntil('\n');
    debugln("tmp:" + tmp);
    if(isSignedNumeric(tmp))
    {
      if(tmp.startsWith("-"))
      {
          sign = -1;
          tmp.replace("-","");
      }
      encoderPosition = tmp.toInt();
      encoderPosition *= sign;
    }
    else //We will come here is we have recieved some input from encoder but it has junk characters in it.
    {
      //As a hack update the timestamp here so that the curtain doesnt auto stop, hoping we will get a valid encoder input soon enough
      //If this assumption is not true then we have a potential problem !!!!!!!!!!!WARNING !!!!!!!!!!!!!!!!!!!!!!!!!
      lastEncoderTime = millis();//update the timestamp as we've received a change in encoder value
    }
    //Serial.println(encoderPosition);
  }
}

void ISRCurtainUp()
{ btnUp = true;}

void ISRCurtainDn()
{ btnDn = true; }

void ISRCurtainTop()
{ btnTop = true; }

void curtainUp()
{
  if(!btnTop)
  {
    if((currentStepPosition > 0) || crashRecovery)// TO DO do direction check here
    {
      curtainDirectionUp = true;
      prepCurtainMove();
      myservo.write(180);
      lastEncoderTime = millis();//initialize the last encoder time as we're starting to move now.
      debugln("Curtain is moving up");
    }
    else
      debugln("Curtain up command ignored as curtain is at the top");
  }
  else
      debugln("Curtain up command ignored as curtain is at the extreme top");
}
  
void curtainDn()
{
  if((currentStepPosition < curtainMaxSteps) || crashRecovery) // TO DO do direction check here
  {
    curtainDirectionUp = false;
    prepCurtainMove();
    myservo.write(0);
    lastEncoderTime = millis();//initialize the last encoder time as we're starting to move now.
    debugln("Curtain is moving down");
  }
  else
    debugln("Curtain down command ignored as curtain is at the bottom");
}

void prepCurtainMove()
{
    servoRunning = true;
    //save the current state so that if it crashes before stopping cleanly we know that it crashed in between running
    state curr_state = {.steps = currentStepPosition, .isRunning = servoRunning, .maxSteps = curtainMaxSteps};
    saveCurrentState(curr_state);
    if(!myservo.attached())
      myservo.attach(MOTOR_PIN);
    lastEncoderTime = millis();//initialize the last encoder time as we're starting to move now.
}

void curtainStop()
{
    if(myservo.attached())
      myservo.detach();
    servoRunning = false;
    delay(1000);//wait for a second for the motor to stop completely
    updateEncoder();//update the final stopped encoder position 
    state curr_state = {.steps = currentStepPosition, .isRunning = servoRunning, .maxSteps = curtainMaxSteps};
    saveCurrentState(curr_state);
    debugln("Curtain stopped at position:");debugln(String(currentStepPosition));
}

//saves the current state of the curtain position to the memory
//The file contains one line in the form "200,200,0" where first one is current step position, second is the max steps the curtain can move and third is the status of the running status of the curtain
//the running status is set to 1 when the curtain is moving and set to 0 when it has been successfully stopped. If there is a crash or power disruption while moving the curtain this will remain 1 and hence we'll 
//consider this as a crash recovery
void saveCurrentState(state this_state)
{
  //Save the current values only if we are not in crash recovery mode else it will override the crash recovery mode in the config.
  if(crashRecovery)
    return;
  
  //To save on file writes compare the last state saved to the current state, if there is no change, don't save
  if(last_state.steps == this_state.steps && last_state.maxSteps == this_state.maxSteps && last_state.isRunning == this_state.isRunning)
    return;
  
  String strFile = "";
  strFile += String(this_state.steps) + "," + String(this_state.maxSteps) + "," + String(this_state.isRunning);
  
  File logF = SPIFFS.open("/curtain.txt", "w");
  if (logF) 
  { 
    logF.print(strFile);
    logF.close();
    //save the current state into the last state variable to compare later
    last_state.steps = this_state.steps;last_state.maxSteps = this_state.maxSteps;last_state.isRunning = this_state.isRunning;
    debugln("Current state saved to the file");
  }
  else
  {
    debugln("Error opening file. Can't write the status of the curtain");
  }
}

//dont call this method again somewhere except in setup() as I have SPIFFS.begin inside it. I am not sure what would happen if this method is called repeatedly
//The file contains one line in the form "200,200,0" where first one is current step position, second is the max steps the curtain can move and third is the status of the running status of the curtain
state getCurrentState()
{
  state curr_state = {.steps=0, .isRunning=false, .maxSteps = curtainMaxSteps};
  if (!SPIFFS.begin()) 
     while(1);
  File f = SPIFFS.open("/curtain.txt", "r");
  delay(10);
  if (f) 
  {
    String file_str;
    //String read back is of the form <steps>,<isRunning><maxSteps> , eg. 45,0,800 => Here 45 is steps and 0 means curtain is not running , 1 would mean its running , and 800 is the max no of steps
    file_str = f.readString();//Reads the entire string unless it encounters the hex character 0x00
    f.close();
    
    char * str = new char [file_str.length()+1];
    strcpy (str, file_str.c_str());
    String tmp = "";

    //get the Steps
    tmp = strtok (str,",");
    if(isNumeric(tmp))
      curr_state.steps = (short)tmp.toInt();
    
    //get the max Steps
    tmp = strtok (NULL,",");
    if(isNumeric(tmp))
      curr_state.maxSteps = (short)tmp.toInt();

    //Get the isRunning flag 
    tmp = strtok (NULL, ",");
    
    if(isNumeric(tmp))
      curr_state.isRunning = (tmp.toInt() == 0?false:true);
  }
  else
    debugln("Error opening file: /curtain.txt");
  return curr_state;
}


//********* http handler functions ***********************

//sample request: http://192.168.1.10/up
void handleCurtainUp()
{
  curtainUp();
  httpServer->send ( 200, "text/html", "Curtain moving up");
}

//sample request: http://192.168.1.10/dn
void handleCurtainDn()
{
  curtainDn();
  httpServer->send ( 200, "text/html", "Curtain moving down");
}

//sample request: http://192.168.1.10/st
void handleServoStop()
{
  curtainStop();
  httpServer->send ( 200, "text/html", "Curtain stopped");
}

void handleRoot()
{
  httpServer->send ( 200, "text/html", rootHTML);
}

//sample request: http://192.168.1.10/version
void handleVersion()
{
  httpServer->send ( 200, "text/html", "<html><body><B>" + WiFi.hostname() + "</B></BR><p>Version " + String(VERSION) + "</p><p>ESP Uptime(hr:mn:sec): " + formatTime(millis()/1000) + "</p></body></html>" );
}

//sample request: http://192.168.1.10/rst
void handleReset()
{
  //This should only be called when the curtain is at its bottom
  currentStepPosition = curtainMaxSteps;
  //encoderPosition = 0;
  //lastEncoderPosition = 0;
  crashRecovery = false;//this is to enable saving of current configuraiton
  curtainStop();//although I call call saveCurrentState here but calling stop to be safe in case I get the reset command while servo is running, curtainStop in turn will save the current state.
  debugln("Reset successfully");  
  httpServer->send ( 200, "text/html", "Curtain position reset successfully");
}

//sample request: http://192.168.1.10/sav?pos=0&run=0&max=100
void handleSaveState()
{
  String strPos = "",strRunning = "",strMax ="";
  strPos = httpServer->arg("pos");
  strRunning = httpServer->arg("run");
  strMax = httpServer->arg("max");
  
  if( strPos!= "" && (strRunning== "0" || strRunning == "1") && isNumeric(strPos) && strMax!= "" && isNumeric(strMax) )
  {
    currentStepPosition = (short)strPos.toInt();
    curtainMaxSteps = (short)strMax.toInt();
    state curr_state = {.steps = currentStepPosition, .isRunning = (strRunning=="0"?false:true),.maxSteps = curtainMaxSteps};
    saveCurrentState(curr_state);
    httpServer->send ( 200, "text/html", "Successfully saved state");
  }
  else
    httpServer->send ( 200, "text/html", "Failed to saved state, invalid arguments:" + strPos + "," + strRunning);
  
}

//sample request: http://192.168.1.10/csh
void handleCrash()
{
  crashRecovery = true;//this is to enable saving of current configuraiton
  debugln("Crash recovery mode set to true");  
  httpServer->send ( 200, "text/html", "Successfully entered crash recovery mode");
}


//sample request: http://192.168.1.10/dbg
void handleDebug()
{
  String strHTML =  "<html><body>";
  strHTML += "<p>Step Position: ";
  strHTML += String(currentStepPosition) + "</p>";
  strHTML += "<p>Encoder Position: ";
  strHTML += String(encoderPosition) + "</p>";
  strHTML += "<p>Crash Recover mode: ";
  strHTML += String(crashRecovery) + "</p>";
  strHTML += "<p>max Steps: ";
  strHTML += String(curtainMaxSteps) + "</p>";

  strHTML += "</body></html>";
  httpServer->send ( 200, "text/html",  strHTML);
 
}

//********* Utility functions ***********************
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
  debugln("str.length2=" + String(str.length()));
  for(unsigned short i=0;i<str.length();i++)
    if(!isDigit(str.charAt(i)))
      {debug("false");
      return false;}
    else
      {debug("true");}
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


//Logs a debug message without newline
void debug(String msg)
{
#ifdef DEBUG
    Serial.print(msg);
#endif
}

//Logs a debug message with newline
void debugln(String msg)
{
#ifdef DEBUG
    Serial.println(msg);
#endif
}
