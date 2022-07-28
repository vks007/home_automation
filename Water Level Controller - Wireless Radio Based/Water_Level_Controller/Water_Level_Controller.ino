/*
 * This program is a water level controller using ATiny 2313A chip. It offers the following functionalities.
 * The program receives its inputs like overhead water level/sump level/manual switches etc and outputs a logic HIGH/LOW to be used to turn the motor ON/OFF
 * It also putputs on other PINs to indicate the status of various things
 * Requirements:
1.It should automatically start the motor if the water level drops below a certain minimum in the overhead tank
2.It should stop the motor if the water level reaches a certain maximum in the overhead tank
3.It should not start the motor if the water is below a certain minimum in the sump
4.It should have a manual and auto mode, in manual mode motor will not start automatically nor stop automatically. It wont even take into account the sump water level
5.It should have a timer function so that the motor will stop running irrespective of the water level beyond that time - this is to prevent motor running indefinitely due to the program not functioning well
6.It should have a Stop switch (manual override) which will stop the motor if pressed - this will work both in Auto & Manual mode
7.If there is no reception between the Tx & Rx AND motor is already ON then it should check for a few minutes before stopping. If it is already OFF then it should remain OFF. This can then be reset by start/stop switch

 */


//Includes

//#defines
#define MAX_UPTIME  90 //Maximum time for which motor would remain ON before turning OFF as a precaution (in minutes)
#define MAX_SIGNAL_TIME_CTR  20000 //Max loop counter over which a Rx signal is checked

//Global variables
//Pins for ATtiny2313
short tankMinPin = 7;//Physical pin 9 on 2313, pin for min level of overhead tank
short tankMaxPin = 6;//Physical pin 8 on 2313, pin for max level of overhead tank
short sumpMinPin = 5;//Physical pin 7 on 2313, pin for min level of sump tank (underground tank from which water is pumped)
short manualStartStopPin = 4;//Physical pin 6 on 2313,pin for Start / Stop button (works both in manual and auto mode)
short autoModePin = 3;//Physical pin 5 on 2313, pin to indicate auto or manual mode
short motorModePin = 8;//Physical pin 11 on 2313, pin to turn ON the motor
short rxSignalPin = 2;//Physical pin 4 on 2313, pin to indicate the Rx signal status from the Tx. This is a blinking status
    // It becomes high only when actually receiving signal. My Tx transmits a signal for around 300ms in a 2 sec cycle
//short debugPin1 = 9;//Physical pin 12 on 2313, pin to debug info

/*
//Pins for Arduino Uno
short tankMinPin = 2;//pin for min level of overhead tank
short tankMaxPin = 3;//pin for max level of overhead tank
short sumpMinPin = 4;//pin for min level of sump tank (underground tank from which water is pumped)
short manualStartStopPin = 5;//pin for Start / Stop button (works both in manual and auto mode)
short autoModePin = 6;//pin to indicate auto or manual mode
short motorModePin = 7;//pin to turn ON the motor
*/

//Initialize status of all inputs so that the motor would generally remain OFF initially
short tankMinStatus = HIGH;//This will ensure motor starts in OFF mode
short tankMaxStatus = HIGH;//This will ensure motor starts in OFF mode
short sumpMinStatus = LOW;
short autoModeStatus = HIGH;// By default we are in Auto Mode
short lastButtonState = LOW;
boolean motorRunning = false;
boolean autoCutOff = false;
boolean rxSignalStatus = false;

unsigned long motorStartTime = 0;//counts the milliseconds since motor was started , in ms
unsigned long minElapsed = 0;//stores the time since the motor has been ON , in minutes

void setup() {
// put your setup code here, to run once:
//By default all Pins are INPUT so no need to declare them so
pinMode(motorModePin,OUTPUT);
//pinMode(debugPin1,OUTPUT);

//Initialize the motor to be OFF when the program starts
//This may not be necessary as the PIN would be LOW by default , verify and comment out
digitalWrite(motorModePin,LOW);

// initialize serial communication for debugging
//Serial.begin(9600);

}
/*
void blinkled(short count)
{
    short i=0;
    for(i=0;i<count;i++)
    {
      digitalWrite(motorModePin,HIGH);
      delay(2000);
      digitalWrite(motorModePin,LOW);
      delay(1000);
    }
}
*/

//This function will check if the Rx is able to receive the signal status from the Tx periodically.
/*
 * THe Tx transmits signals in bursts for a few milliseconds every few seconds and hence its important to monitor the signal over at least a minute cycle
 * so that in case we lose the signal for a few cycles, the motor state isnt changed based on that condition.
 */
boolean checkSignalStatus()
{
  //Total time for which this will wait for the high signal on Rx pin = (MAX_SIGNAL_TIME_CTR * delay time)/1000 seconds (currently 300 sec or 5 min)
  short ctr = 0;

  while((digitalRead(rxSignalPin) == LOW) && ctr < MAX_SIGNAL_TIME_CTR)//We dont have a Rx signal, keep trying until you timeout
  {
    delay(10);//sleep for some time and try again
    ctr++;
  }
  if(ctr >= MAX_SIGNAL_TIME_CTR)
    return false;//We were not able to get a high signal on the Rx Pin - the link b/w Tx & Rx is broken
  else
    return true;//Success
}

void loop() {
  
  // put your main code here, to run repeatedly:
  //Collect the values of the inputs
  sumpMinStatus = digitalRead(sumpMinPin);
  autoModeStatus = !digitalRead(autoModePin);
  
  //If a manual start has been pressed then turn ON the motor. This will sustain itself on looping except if sumpMinStatus becomes LOW
  if(digitalRead(manualStartStopPin) != lastButtonState)
  {
      if(lastButtonState == LOW)//means manualStartStopPin is HIGH (did not want to read or store it's value once again)
      {
        motorRunning = !motorRunning;//Start or Stop the motor depending on current State
        lastButtonState = HIGH;//Change the state as we have changed the motor state now
        autoCutOff = false;//We break the autoCuttOff only when somebody manually presses the start/stop button. although unncesarry, This doesnt do any harm in stop condition
      //Serial.print("motor State changed. state = ");
      //Serial.println(motorRunning);
      }
      else
        lastButtonState = LOW;//invert the last button state, this will ensure that we trigger a change in state only on a rising edge ie. when button passes state from LOW to HIGH and not
        //when it passes HIGH to LOW (trailing edge)
  }


  //If auto cut off timer has worked then the auto logic of the controller should not work, because that's the intent. Only when somebody manually presses
  //start or resets does it start working again
  if(autoModeStatus == HIGH && !autoCutOff) //means it is auto mode and auto timer Cut Off has not happened
  {
//    rxSignalStatus = checkSignalStatus(); //Proceed to apply logic based on inputs only if you have a signal from the Tx.
    if(checkSignalStatus())
    {
      //Now that we know the Rx status is okay it is safe to collect the values of the tank levels
      tankMinStatus = digitalRead(tankMinPin);
      tankMaxStatus = digitalRead(tankMaxPin);
      if(motorRunning == true)
        {
          if(sumpMinStatus == LOW)//If water lower than min turn OFF 
          {
            motorRunning = false;
            //Serial.println("sumpMin is LOW , motorRunning set to false");
          }
          else
          {
            if(tankMaxStatus == HIGH)//If tank level is above max turn OFF
            {  
              motorRunning = false;
            //Serial.println("tank is High, motorRunning set to false");
            }
            //Serial.println("Let the motor Run till tank full");
            //else motorRunning remains true and we exit
          }
        }
        else //motorRunning == false
        {
          // Start the motor if it meets the below criteria
          if(tankMinStatus == LOW && sumpMinStatus == HIGH && tankMaxStatus == LOW)
          {
            motorRunning = true;
            //Serial.println("Water less than min, motorRunning set to true");
          }
          else //motorRunning remains false
          { 
            motorRunning = false;
            //Serial.println("Conditions not met, motorRunning set to false");
          }
        }
    }
    else
    {
      motorRunning = false;// Turn OFF the motor as we dont have Tx signals, there's nothing we can do except wait for the signals to be restored
      //Serial.println("No Tx signals , motorRunning set to false");
      //TO DO : build in an alarm system to warn that we've lost the signal
    }
  }

  //This is a emergency timer to avoid burning out of motor if water continually leaks and motor never turns OFF.
  //With this logic the motor would auto cut off after a certain max time defiend by MAX_UPTIME irrespective of other input values
  if(motorRunning == true && minElapsed < MAX_UPTIME)//Continue only if max time has not elapsed, since the motor turned ON
  {
    minElapsed = (millis() - motorStartTime)/60000; //convert into munites
    //Serial.print("minElapsed = ");
    //Serial.println(minElapsed);
    if(digitalRead(motorModePin) == LOW)//Only write a HIGH if the pin is low, this is to identify the first time start condition in the loop
    {
      //Motor is starting for the first time so reset the start time
      motorStartTime = millis();
      digitalWrite(motorModePin,HIGH);//This is the only place where the motor is being turned ON in the entire program
      //Serial.println("Starting motor...");
    }
    //else nothing to do, motor continues to be ON
  }
  else
  {
    
    if(minElapsed >= MAX_UPTIME)//If we came here because max time elapsed then reset the variables related to auto cut off and timer
    {
      autoCutOff = true;//Once auto cut off it can be reset by pressing the manual Start Stop switch (manualStartStopPin)
      motorRunning = false;
      //Serial.println("Auto cutt off time reached , motorRunning set to false");
      minElapsed = 0;
      motorStartTime = millis();//this is necessary so that elapsed time is always a small number, else it will grow, not sure if this is a problem though
      //note that millis() continues to grow till it reaches 50 days and then resets, I hope ATtiny2313 also has the same behavior
    }
    if(digitalRead(motorModePin) == HIGH)//Only write a LOW if the pin is HIGH, this is to identify the time when we are turning OFF a running motor
    {
      digitalWrite(motorModePin,LOW);
      //Serial.println("Stopping motor ...");
    }
      //Serial.println("Motor is in stopped condition");
  }
    
  //Sleep for some time
  delay(100);
  
}

