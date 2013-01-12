//Arduino WIFLY Mini
//now with FPV
//Two-Way Communications
//Direct SPI-UART manipulation.
//
//ADDED ACCELEROMETER FEEDBACK, REMOVED pot reading/logging, DUE TO MEMORY ISSUES.
//removed complicated braking functions.
//simplified communications method, now transmissions to (and from) the wifly are bundled, to more resemble a PPM stream
float AWMV = 1.110;   //arduino wifly mini version (AWMV).
 
#include "WiFly.h" // We use this for the preinstantiated SpiSerial object.
#include <Servo.h>
 
//WIFLY SETTINGS:
boolean performSetup = false; //should we set these settings? or skip them?
boolean requireFirstContact = false; // do we need to make first contact with AP before entering main loop?
//String hostIP = "192.168.1.4";  //mbp, wireless, Hitch
String hostIP = "192.168.1.6";  //Blair WIRED, Hitch
String ssid = "Hitch";            //WRT54GL
String phrase = "TwentyTwo";
String listenPort = "8888";
String sendPort = "6000";
 
//ARDUINO SETTINGS AND VARIABLES:
//PIN DEFINITIONS
const int potPin = A0;    //potentiometer input pin.
const int pinAccel1X = A1;    //accelerometer #1 Xplane output.
const int pinAccel1Y = A2;    //accelerometer #1 Yplane output.
const int pinAccel1Z = A3;    //accelerometer #1 Zplane output.
const int pinInfoLED = 8; //pin on which information LED is found.
const int pinOpto = 4; //opto-isolator pin, for turning on ESC.
const int pinThrottle = 5; //throttle/drive output
const int pinSteer = 6; //the pin that writes location to steering servo.
//SERVOS
Servo theCamera;  //this variable references the steering servo.
Servo theWheel;  //this variable references the steering servo.
Servo theGas; //the variable for referencing the ESC.
//SERVO-AT-REST SETTINGS
int cameraCentre = 90; //default centre position of camera servo.
int wheelCentre = 92; //default wheel center. Official value comes from processing.
//SERVO RANGE SETTINGS (Movement, Allowances)
//steering
int steeringRange = 8; //default steering range.
int middleRange = 4;   //how far on either side of wheelCentre will the arduino correct for a constant feedback from the steering servo pot?
int autoCameraRange = 50; //default movement degrees either side of center for camera servo.
//throttle
int throttleNeutral = 50; //no throttle. SAFETY VALUE.
int throttleFullReverse = 30;  //be careful with this.
int throttleSetting = throttleNeutral; //this is for Arduino's reference when error handling, etc.
//camera
//int cMMdelay = 3300; //cameraMoveManualDelay, delay in milliseconds before arduino reverts back to auto camera movement, if set.
//unsigned long cMMtimeout = millis();
//
//SERVO Bearing/Setting Memory and Variables
//steering
//int cBearing; //this is the currently-set steering servo bearing.
//int sBearing; //this is the setting of the servo guessed at by the arduino; a rough estimate.
//unsigned long sStime;   //steering servo time; if millis is beyond this, then tick the servo one degree toward cBearing, then reset.
//unsigned long oldBearing;   //when this is up, then the bearing is considered old. move back to strict pot threshold.
//FORCE FEEDBACK: Settings, Values, Ranges, Allowances, Variables
unsigned long accelCheckTime;
int accelCheckDelay = 33; //milliseconds between accel checks.
int accel1counter = 0;  //keeps track of where we are in the accel1 average arrays, used for determining what the average of the last 100 readings have been.
const int accelAVGarraySize = 12;
int accel1avgXarray[accelAVGarraySize];
int accel1avgYarray[accelAVGarraySize];
int accel1avgZarray[accelAVGarraySize];
boolean FFaccelsCalibrated = false; //are the accelerometers calibrated?
boolean FFsendACCELDATA = false;   //should the arduino check the ACCELEROMETER(s) and send back information to the driver?
int minAccelFFamplitudeX = 27; //default sent by host at calibrate sensors (or send thresholds).
int minAccelFFamplitudeY = 5; //default... sent again by host at calibrate sensors.(or send thresholds).
int minAccelFFamplitudeZ = 27; //default... sent again by host at calibrate sensors.(or send thresholds).
int lastAXdifference = 0;  //what was the last accelerometer difference?
int lastAYdifference = 0;  //what was the last accelerometer difference?
int lastAZdifference = 0;  //what was the last accelerometer difference?
int lastAXFFresult = 0; //what was the last aff result?
int lastAYFFresult = 0; //what was the last aff result?
int lastAZFFresult = 0; //what was the last aff result?
//WIFLY COMMUNICATION WITH ARDUINO
//heartbeat
int heartRate = 888; //heartbeat delay in milliseconds.
unsigned long deadBeat; //this is theTime plus the heartRate delay. If no HB signal is received before deadBeat time is up, consider connection lost and go into errorHandling.
unsigned long flatlinedAt;
unsigned long timeToWiflyReset;
int flatlineWiflyResetDelay = 27000;
boolean resetAfterFlatline = false;
//register availability (applies to specific functions, to stop overflowing the buffer). 
boolean rCRV = false; // this determines if certain functions are allowed to send data to the SPI register.
unsigned long cSW; //time until another variable can be sent to the register.
int cSD = 8; //the delay, in milliseconds, before another character can be sent to the SPI register.
//states
boolean flatline = false;
int wFFC = 0; //wait for first connection, counter.
boolean icM = false; //Initial Connection Made? Have we connected to an AP?
boolean commandMode = false;   //is the Wifly presently in command mode?
boolean started = false; //did we receive the start command, and are now started? (will change if connection is lost or started is received again, or if there's an error.).
boolean escOn = false; //same as started, but for ESC. Is it on?
boolean autoCamera = false; //does the camera movement follow the steering?
boolean settingsReceived = false;
boolean autoCameraProportional = false;
//data
int sCS = 0; //settings checksum.
char newInsCharArray[20]; //new instruction char array... stores the instruction.
int iC = 0; //instruction counter, for the newInsCharArray (new instruction char array).
String newInstruction;
int cIs = 0; //is the new command an integer, a no value command, or a float? (0,1,2 respectively).
//command counter variables
//I use these to count certain instructions I send more than once to activate.
//I do this when certain functions get set accidently by extraneous data coming
//from the SPI uart.
int CCraf = 0;   //command counter, reset after flatline.
//TIME
unsigned long theTime = millis(); //this will be equated to millis();
 
 
 
 
 
 
 
 
/////////////////////////////////////////////
//                                         //
//          Arduino Setup                  //
//                                         //
/////////////////////////////////////////////
//Setup Routine:
void setup() {
 Serial.begin(9600);
 //Serial.println(" ");
 Serial.print("WIFLY Arduino Mini v");
 Serial.print(AWMV);
 Serial.println(" ");
 Serial.println("--------------------------------------");  
 Serial.println("Initialize SpiSerial.");
 SpiSerial.begin();
 Serial.println("Connected to SPI UART.");
 Serial.println();
 
  //pinMode(pinOpto1, OUTPUT); //SERVO opto-isolator pin set to OUTPUT
  pinMode(pinOpto, OUTPUT); //ESC opto-isolator pin set to OUTPUT
  //digitalWrite(pinOpto1, LOW); //turn off SERVOS
  digitalWrite(pinOpto, LOW); //turn off ESC
 
  pinMode(pinInfoLED, OUTPUT); //information LED pin set to OUTPUT.
  digitalWrite(pinInfoLED, LOW); //information LED off.
 
  theCamera.attach(3);  //associates theCamera with indicated pin.
  theGas.attach(5);  //associates theGas with indicated pin.
  theWheel.attach(6);  //associates theWheel with indicated pin.
 
  theCamera.write(cameraCentre);
  theWheel.write(wheelCentre);
  theGas.write(throttleNeutral);
  //sBearing = wheelCentre; //initialize sBearing
 
  if(requireFirstContact == true) {
    icM = false;
  } else {
    icM = true;
  }
 
  if(performSetup == true) {
    //configure wifly
    configureWifly();
  } else {
    //do nothing
  }
 
  //Serial.println("DONE ARDUINO SETUP");   
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
//////////////////////////////////////
//                                  //
//          ERROR HANDLING          //
//                                  //
//////////////////////////////////////
 
void errorHandle(String theError)
{
  //do nothing for the time being.
  if(theError == "flatline") {
    if(throttleSetting > (throttleNeutral + 10)) {
      throttle(throttleFullReverse);
      delay(777);
    }
    neutral();
    if(escOn == true) {
      ESC();
    }
    if(started == true) {
      start();
      flatline = true;
      flatlinedAt = millis();
      timeToWiflyReset = flatlinedAt + flatlineWiflyResetDelay;
      Serial.println("Flatlined!");
    }
 
  }
}
 
 
 
 
 
 
 
 
 
////////////////////////////////////////////////////
//                                                //
//     Update Time Sensitive Functions (uTSF)     //
//     (Sensor checks, etc)                       //
//                                                //
////////////////////////////////////////////////////
void checkACCEL1() {
  theTime = millis();
  if(theTime > accelCheckTime) {
     int accelX = analogRead(pinAccel1X);    // read accel's X value
     int accelY = analogRead(pinAccel1Y);    // read accel's Y value
     int accelZ = analogRead(pinAccel1Z);    // read accel's Z value
     //minAccelFFamplitude
     int accelXaverage = 0;
     int accelYaverage = 0;
     int accelZaverage = 0;
     accel1avgXarray[accel1counter] = accelX; //put X reading into array
     accel1avgYarray[accel1counter] = accelY; //put Y reading into array
     accel1avgZarray[accel1counter] = accelZ; //put Z reading into array
     accel1counter++;
     if(accel1counter > accelAVGarraySize) {
       accel1counter = 0;
     }
     for(int a = 0; a<accelAVGarraySize; a++) {
         //add together, to update average...
         accelXaverage = accelXaverage + accel1avgXarray[a];
         accelYaverage = accelYaverage + accel1avgYarray[a];
         accelZaverage = accelZaverage + accel1avgZarray[a];
     }
     accelXaverage = accelXaverage / accelAVGarraySize;
     accelYaverage = accelYaverage / accelAVGarraySize;
     accelZaverage = accelZaverage / accelAVGarraySize;
 
     //now find out if there's enough of a disturbance to report...
     int aXdifference = 0;
     int aYdifference = 0;
     int aZdifference = 0;
     //determine if X reading is out of range
     int accelUnder = accelXaverage - minAccelFFamplitudeX;
     int accelOver = accelXaverage + minAccelFFamplitudeX;
     if(accelX <= accelUnder) {
       aXdifference = accelXaverage - accelX;
     } else if (accelX >= accelOver) {
       aXdifference = accelX - accelXaverage;
     }
     //determine if Y reading is out of range
     accelUnder = accelYaverage - minAccelFFamplitudeY;
     accelOver = accelYaverage + minAccelFFamplitudeY;
     if(accelY <= accelUnder) {
       aYdifference = accelYaverage - accelY;
     } else if (accelY >= accelOver) {
       aYdifference = accelY - accelYaverage;
     }
     //determine if Z reading is out of range
     accelUnder = accelZaverage - minAccelFFamplitudeZ;
     accelOver = accelZaverage + minAccelFFamplitudeZ;
     if(accelZ <= accelUnder) {
       aZdifference = accelZaverage - accelZ;
     } else if (accelZ >= accelOver) {
       aZdifference = accelZ - accelZaverage;
     }
 
     //Serial.print("accelZ: ");
     //Serial.println(accelZ);
     //delay(500);
 
     boolean bufferLoaded = false;
     //X
     if(aXdifference != lastAXdifference) {
       //the affXresult has changed, send to host.
       ptb(0x3C); //print "<" to spi uart, for X
       wSPIintToChar(aXdifference);
       bufferLoaded = true;
       lastAXdifference = aXdifference;
     }
     //Y
     if(aYdifference != lastAYdifference) {
       //the affZresult has changed, send to host.
       ptb(0x3E); //print ">", for Y
       wSPIintToChar(aYdifference);
       bufferLoaded = true;
       lastAYdifference = aYdifference;
     }
     //Z
     if(aZdifference != lastAZdifference) {
       //the affZresult has changed, send to host.
       ptb(0x5E); //print "^", for Z.
       wSPIintToChar(aZdifference);
       bufferLoaded = true;
       //Serial.println(aZdifference);
       lastAZdifference = aZdifference;
     }
     if(bufferLoaded) {
       fb(); //flush buffer
     }
     accelCheckTime = theTime + accelCheckDelay;
  }
}
void heartbeat()
{
  theTime = millis();
  deadBeat = theTime + heartRate;
  //Serial.println("BEAT");
}
void checkHeart() 
{
  if(started == true) {
    theTime = millis();
    if(theTime > deadBeat) {
      errorHandle("flatline");  
    }
  } else {
    //started is not equal to true.
    //did we flatline?
    if(flatline == true) {
        theTime = millis();
        if((theTime > timeToWiflyReset) && resetAfterFlatline) {
          //in development...
          Serial.println("Attempting to resuscitate connection...");
          wiflyReconnectToAP();
          //wiflySuddenReboot();
        }
    }
  }
}
void uTSF()
{
  checkHeart();
  if(FFsendACCELDATA && FFaccelsCalibrated) {
    checkACCEL1();
  }
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
/////////////////////////////////////////////
//                                         //
//            DATA MANIPULATION            //
//                                         //
/////////////////////////////////////////////
 
//delegate received command.
void delegate(String wtd, char* tVc) {  //tVc = the float char
  float tVf = 0.0;  //the value, float
  int tV = 0;     //integer
 
  if(cIs == 0) {
    //the value needs ot be an integer. most cases.
    tV = atoi(tVc);  //turn char array into an integer.
  } else if (cIs == 1) {
    //there is no accompanying value
    //do nothing.
  } else if (cIs == 2) {
    //the value must be a float
    tVf = atof(tVc); //turn char array into a float.
  }
  //most important first:
  if(wtd == "B") {
      //first tier.
      heartbeat();
      //Serial.println("Heartbeat!");
  } else {  //so, not a heartbeat.
    //second tier.
    if(wtd == "W") {
      //steer
      wheel(tV);
    } else if (wtd == "T" && escOn == true) {
      //throttle
      throttle(tV);
    } else if(wtd == "L") {
      //if(autoCamera) {
        //theTime = millis();
        //cMMtimeout = theTime + cMMdelay; //camera manual move timeout, theTime offset by camera manual move delay.
      //}
      if(!autoCamera) {
        camera(tV);
      }
    } else { 
      //third tier.
      if (wtd == "A") {
        ESC();
      } else if(wtd == "S") {
        start();
      } else if (wtd == "R") {
        //set the steering range.
        //Serial.println("SET STEERING RANGE");
        steeringRange = tV;
        sCS++;
        //Serial.print(steeringRange);
        //Serial.println("");
      } else if (wtd == "V") {
        //set the wheelCentre
        //Serial.println("SET WHEEL CENTRE");
        wheelCentre = tV;
        sCS++;
        //Serial.print(wheelCentre);
        //Serial.println("");
      } else if (wtd == "h") {
        Serial.print("autoCamera: ");
        if(!autoCamera && settingsReceived) {
          autoCamera = true;
          ptb(0x68);  //h
          ptb(0x31);  //indicate on.
          fb();
          Serial.println("ON");
        } else {
          autoCamera = false;
          ptb(0x68);  //h
          ptb(0x30);  //indicate off.
          fb();
          Serial.println("OFF");
        }
      } else if (wtd == "a") {
        Serial.println("SET autoCameraRange: ");
        autoCameraRange = tV;
        sCS++;
        Serial.println(cameraCentre);
      } else if (wtd == "l") {
        Serial.println("SET Camera Centre: ");
        sCS++;
        cameraCentre = tV;
        Serial.println(cameraCentre);
      } else if (wtd == "x") {
        //Serial.println("SET ACCELEROMETER MINIMUM FF AMPLITUDE THRESHOLD X");
        minAccelFFamplitudeX = tV;
        sCS++;
        //Serial.println(minAccelFFamplitudeX);
      } else if (wtd == "y") {
        //Serial.println("SET ACCELEROMETER MINIMUM FF AMPLITUDE THRESHOLD Y");
        minAccelFFamplitudeY = tV;
        sCS++;
        //Serial.println(minAccelFFamplitudeY);
      } else if (wtd == "z") {
        //Serial.println("SET ACCELEROMETER MINIMUM FF AMPLITUDE THRESHOLD Z");
        minAccelFFamplitudeZ = tV;
        sCS++;
        //Serial.println(minAccelFFamplitudeZ);
      } else if (wtd == "~" && started == false) {
        //calibrate accelerometer rest ranges
        if(!started) {
          Serial.println("Calibrating Accelerometer");
          calibrateAccelerometer();
        }
      } else if (wtd == "M") {
        //we want pots data.
        Serial.println("Settings checksum...");
        if(tV == 0) {
          Serial.println("Cleared.");
          sCS = tV;
        } else if (tV > 0) {
          Serial.print("Comparing, sending to HOST. sCS = ");
          Serial.println(tV);
          ptb(0x4D);  //M
          wSPIintToChar(sCS);
          fb();
        }
      } else if (wtd == "m") {
        //settings checksum checked out.
        Serial.println("Checksum Confirmed. Settings Received.");
        settingsReceived = true;
      } else if (wtd == "X") {
        //we want acceleration data.
        if(FFsendACCELDATA && FFaccelsCalibrated) {
          FFsendACCELDATA = false;
          //send ACK
          ptb(0x58);  //X
          ptb(0x30);  //indicate off.
          fb();
        } else if (FFaccelsCalibrated) {
          Serial.println("SEND FF ACCELERATION DATA");
          FFsendACCELDATA = true;
          //send ACK
          ptb(0x58);  //X
          ptb(0x31); //indicate on.
          fb();
        }
      } else if (wtd == "Z" && started == false) {
        //set pot defaults.
        if(CCraf < 2) {
          CCraf++;
        } else {
          CCraf = 0;
          if(resetAfterFlatline) {
            resetAfterFlatline = false;
            ptb(0x5A); //Z
            ptb(0x30); //indicate off.
            fb();
          } else if (!resetAfterFlatline) {
            resetAfterFlatline = true;
            ptb(0x5A); //Z
            ptb(0x31); //indicate on.
            fb();
          }
        }
      } else {
        //clear counted commands to avoid unexpected results
        CCraf = 0; 
        //CCpotvals = 0;
      }
    }
  }  //end of not a heartbeat if
 
  //Serial.print(wtd);
  //Serial.println(tV);
}
//interpret received data
void interpret(char c)
{
  String iD = (String)c; //incoming data equated to c.
  //Serial.println(iD);
  if((iD != "0") && (iD != "1") && (iD != "2") && (iD != "3") && (iD != "4") && (iD != "5") && (iD != "6") && (iD != "7") && (iD != "8") && (iD != "9") && (iD != ".")) {
    if(newInstruction != "") {
      delegate(newInstruction, newInsCharArray);    //send instruction to be delegated.
      //Serial.println(newInstruction);
    }
    newInstruction = iD;
    cIs = 1;  
    iC = 0;                         //set instruction counter to zero.
    memset(newInsCharArray,0,sizeof(newInsCharArray)); //clear char array.
  } else if (newInstruction != "") {
    //accumulate contents/details of new instruction.
    newInsCharArray[iC] = c;
    iC++;
    //there are accompanying values, set cls to 0
    if(cIs == 1) {
      cIs = 0;
    }
    if(cIs == 1 && iD == ".") {
      //accompanying value will be a float
      cIs = 2;
    }
    //Serial.print(c);
  } else {
    //something's happened that shouldn't have.
    Serial.println("ComSeqErr");
    errorHandle("ComSeqErr");
  }
 
}
void wSPIintToChar(int tpd) {
  if(rCRV == true) {
    char tpdc[12];  //was six
    itoa(tpd, tpdc, 10);  //turn the integer into an ascii string/char array.
    int tpdcl = strlen(tpdc);  //how long is the string?
    for(int ttt = 0; ttt < tpdcl; ttt++) {
      //Serial.println(tpdc[ttt]);
      ptb(tpdc[ttt]);
    }
  }
}
 
 
 
 
 
 
 
 
 
/////////////////////////////////////////////////////////////
//                                                         //
//                    COMMAND   FUNCTIONS                  //
//                                                         //
/////////////////////////////////////////////////////////////
void neutral()
{
  throttle(throttleNeutral);
}
void ESC() {
    //turn on the ESC or turn it off.
    if(escOn == false && started == true) {
      ptb(0x41);
      digitalWrite(pinOpto, HIGH); //ESC ON
      escOn = true;
      Serial.println("ESC is on");
    } else if(escOn == true) {
      ptb(0x61);
      digitalWrite(pinOpto, LOW); //ESC OFF
      escOn = false;
      Serial.println("ESC is off");
    }
    fb(); //flush buffer
}
void camera(int dir)
{
  theCamera.write(dir);
}
void wheel(int bearing)
{
  theWheel.write(bearing);
 
  //cBearing = bearing;
  if(autoCamera) {
    //theTime = millis();
    //if(theTime > cMMtimeout) {
      float cameraPoint = 0.0;
      int cameraPointSet = 0;
      float percentToMove = 0.0;
      if(bearing < wheelCentre) {
        //turning left
        percentToMove = ((float)wheelCentre - (float)bearing) / (float)steeringRange;
        if(autoCameraProportional) {
          percentToMove = pow(percentToMove,2);
        } else {
          //do nothing.
        }
        cameraPoint = (float)cameraCentre + ((float)autoCameraRange * percentToMove);
      } else if (bearing > wheelCentre) {
        percentToMove = ((float)bearing - (float)wheelCentre) / (float)steeringRange;
        if(autoCameraProportional) {
          percentToMove = pow(percentToMove,3);
        } else {
          //do nothing.
        }
        cameraPoint = (float)cameraCentre - ((float)autoCameraRange * percentToMove);
      } else {
        //camera centred.
        cameraPoint = (float)cameraCentre;
      }
      cameraPointSet = cameraPoint; //float to int.
      //Serial.print("cameraPointSet: ");
      //Serial.println(cameraPointSet);
      camera(cameraPointSet);
    //}
  }
 
}
void throttle(int power)
{
  throttleSetting = power;
  theGas.write(power);
}
void start() {
    if(started == false) {
      if(settingsReceived) {
        //start up, send ACK.
        started = true;
        ptb(0x53);
        Serial.println("STARTED");
        heartbeat(); //start the heart.
        flatline = false;
      } else {
        //request user send settings
        ptb(0x70);  //ask for settings ("p");
        fb();       //flush buffer
      }
    } else if (started == true) {
      //shut down, send ACK.
      started = false;
      ptb(0x73);
      digitalWrite(pinOpto, LOW); //ESC OFF
      escOn = false;
      Serial.println("STOPPED");
    }
    fb();
}
void calibrateAccelerometer() {
  //set accelerometer at-rest values...
  int theXaverage = 0;
  int theYaverage = 0;
  int theZaverage = 0;
  for(int a = 0; a < accelAVGarraySize; a++) {
    //initialize array
    accel1avgXarray[a] = analogRead(pinAccel1X);
    theXaverage = theXaverage + accel1avgXarray[a];
    accel1avgYarray[a] = analogRead(pinAccel1Y);
    theYaverage = theYaverage + accel1avgYarray[a];
    accel1avgZarray[a] = analogRead(pinAccel1Z);
    theZaverage = theZaverage + accel1avgZarray[a];
 
    boolean showme = false;
    if(showme) {
      Serial.print("accel1avgXarray[");
      Serial.print(a);
      Serial.print("] = ");
      Serial.print(accel1avgXarray[a]);
 
      Serial.print(", accel1avgYarray[");
      Serial.print(a);
      Serial.print("] = ");
      Serial.print(accel1avgYarray[a]);
 
      Serial.print(", accel1avgZarray[");
      Serial.print(a);
      Serial.print("] = ");
      Serial.println(accel1avgZarray[a]);
    }
 
    delay(15);
  }
  Serial.println("BEFORE DIVISION:");
  Serial.print("theXaverage = ");
  Serial.print(theXaverage);
  Serial.print(", theYaverage = ");
  Serial.print(theYaverage);
  Serial.print(", theZaverage = ");
  Serial.println(theZaverage);
 
 
  theXaverage = theXaverage / accelAVGarraySize;
  theYaverage = theYaverage / accelAVGarraySize;
  theZaverage = theZaverage / accelAVGarraySize;
 
  Serial.println("AFTER DIVISION:");
  Serial.print("theXaverage = ");
  Serial.print(theXaverage);
  Serial.print(", theYaverage = ");
  Serial.print(theYaverage);
  Serial.print(", theZaverage = ");
  Serial.println(theZaverage);
 
  ptb(0x47);  //tell homebase calibration is complete.
  fb();       //flush buffer
 
  FFaccelsCalibrated = true;
}
 
 
 
 
 
 
 
 
/////////////////////////////////////////////
//                                         //
//        WIFLY  GENERAL COMMANDS          //
//                                         //
/////////////////////////////////////////////
//general command definitions:
char EC = 0x0d;  //(carriage return for entering commands)
char ED = 0x24;  //$
char FBC = 0xd;  //flush buffer command, carriage return.
//general command functions:
void exCom() {
  //execute command on WIFLY when in Command Mode.
 SpiSerial.print(EC, BYTE);  //execute command 
}
void fb()
{
  //FLUSH BUFFER (sends buffer contents to host).
  //a carriage return character (hex 0xd) flushes the WIFLYs buffer and sends the msg
  theTime = millis();
  if(theTime > cSW) {
    //it's ok to send now.
    SpiSerial.print(FBC, BYTE);
    rCRV = false; //register can no longer receive values.
    //set wait period!
    cSW = theTime + cSD;
  }
}
void stSPI(String s) {
  SpiSerial.print(s); //print string s to SPI serial
}
void enterCommandMode()
{ 
  for(int i = 0; i <= 2; i++) {
     SpiSerial.print(ED, BYTE);
  }
  rpSPI();
  commandMode = true;
}
void quitCommandMode()
{
  stSPI("exit"); //Exit Command Mode
  exCom();  //execute command
  delay(333);
  rpSPI();
  commandMode = false;
}
void rebootWIFLY() {
  Serial.println("Rebooting...");
  delay(1333);
  //this is performed when already in command mode
  stSPI("reboot"); //Reboot command
  exCom();  //execute command 
  delay(2000);
  commandMode = false;
}
void wiflySuddenReboot()
{
  //performed if not already in command mode.
  enterCommandMode();
  delay(666);
  exCom();  //execute command, in case there's anything there...
  rebootWIFLY();
  rpSPI();
  delay(666);
}
void wiflyReconnectToAP()
{
  //performed if not already in command mode.
  enterCommandMode();
  delay(500);
  stSPI(" ");
  exCom();  //execute command, in case there's anything there...
  delay(333);
  rpSPI();
  stSPI("leave");
  exCom();  //execute command
  delay(3500);
  rpSPI();
  stSPI("join");
  delay(3500);
  rpSPI();
  delay(3500);
  rpSPI();
  quitCommandMode();
  quitCommandMode();
  delay(666);
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  rpSPI();
  theTime = millis();
  timeToWiflyReset = theTime + flatlineWiflyResetDelay;
}
 
 
 
/////////////////////////////////////////////
//                                         //
//         WIFLY SETUP and STATUS          //
//                                         //
/////////////////////////////////////////////
//wifly status manipulation/checking
void waitForFirstConnection()
{
  if(commandMode) {
    String cm = "show connection";
    SpiSerial.print(cm); //show connection status
    exCom();
    delay(666);
    int responseLength = 3; //number of digits of the response minus one, since 0 is counted.
    int i = 0;
    int ii = 0;
    int iStart = cm.length() + 3;
    int iEnd = iStart + 3;
    //Serial.println(iStart);
    //Serial.println(iEnd);
    char c;
    char w[8];
    while(SpiSerial.available() > 0) {
      c = SpiSerial.read();
      if((i>=iStart) && (i<=iEnd)) {
        w[ii] = c;
        ii++;
      }
      i++;
    }
    w[4] = 0x0; //nullify the last character in the string so the comp knows where the end of the string is.
    String iString1 = w;
    if(iString1 == "8130") {
     Serial.println("First AP Association Established. Quitting command mode...");
     quitCommandMode();
     Serial.println(" ");
     Serial.println("Entering Main Loop.");
     icM = true;
    }
    rpSPI();
    delay(1111);
  } else {
    enterCommandMode();
    delay(1111);
    rpSPI();
  }
}
void configureWifly()
{
    delay(333);
    Serial.println("Configuring Wifly...");
    //SET UP WIFLY
 
    //enter command mode (prints $$$).
    enterCommandMode();
 
    rpSPI();
 
    stSPI(" ");   //purposely sabotage the command line, then
    exCom();      //execute command
                  //that would happen if the arduino was rebooted before it had finished issuing commands to WIFLY
 
    //set dhcp mode
    stSPI("set ip dchp 1"); //set DHCP mode
    exCom();      //execute command
 
    rpSPI();
 
    //set the SSID
    stSPI("set wlan ssid "); //set ssid
    stSPI(ssid); //set ssid
    exCom();      //execute command
 
    rpSPI();
 
    stSPI("set wlan phrase "); //set password
    stSPI(phrase); //set ssid
    exCom();      //execute command
 
    rpSPI();
 
    stSPI("set ip host "); //set the host IP address.
    stSPI(hostIP);
    exCom();      //execute command
 
    rpSPI();
 
    stSPI("set ip localport "); //set the listen port
    stSPI(listenPort);
    exCom();      //execute command
 
    rpSPI();
 
    stSPI("set ip remote "); //set the send port
    stSPI(sendPort);
    exCom();      //execute command
 
    rpSPI();
 
    stSPI("save"); //save settings
    exCom();      //execute command
 
    rpSPI();
 
    Serial.println(" ");
    Serial.println("Configured, Saved, Now Reboot:");
    Serial.println(" ");
 
    //reboot the wifly. this function has a delay set within.
    rebootWIFLY();
    rpSPI();
    Serial.println("Wifly is Ready");
}
 
 
 
 
 
///////////////////////////////////////////
//                                       //
//       WIFLY SPI COMMUNICATION         //
//                                       //
//                                       //
///////////////////////////////////////////
void rpSPI()
{
 delay(500);
 while(SpiSerial.available() > 0) {
    Serial.print(SpiSerial.read(), BYTE);
  }
  Serial.println(" ");
}
void ptb(char c)
{
  //prints to SpiSerial (fills up the buffer).
  theTime = millis();
  if(theTime > cSW) {
    if((c == 0x3C) || (c == 0x3E) || (c == 0x5E) || (c==0x4D) || (c==0x4E) || (c==0x68) || (c==0x46)) {
      //we have received a command that requires a value to be sent.
      //therefore, it is ok for the register to receive values.
      //it's MEANT FOR spiIntToChar function... so we don't issue values to the register if there's no representing character.
      rCRV = true;
    }
    //0x4b = K. 0x41 = A. 0x61 = a. 0x53 = S. 0x73 = s.
    //char c = 0x4b; 
    SpiSerial.print(c, BYTE);
  }
}
void listenForCommands()
{
  //read by arduino when receiving commands.
  while(SpiSerial.available() > 0) {
    char c = SpiSerial.read();
    //Serial.print(c);
    interpret(c);  //send char to the interpret function.
  }
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
///////////////
//   LOOP ////
//////////////
 
void loop() 
{ 
  if(icM) {
    listenForCommands(); //Listen for commands from Host received by Wifly.
    uTSF(); //run functions that update time-sensitive variables...
  } else {
    waitForFirstConnection();
  }
}
