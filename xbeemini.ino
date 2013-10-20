
#include <Servo.h> 

Servo servoWheel;
Servo servoThrottle;

//variables
unsigned long theTime = millis();
int wheeldefault_uS = 1500;
int throttledefault_uS = 1473;
String usbInstructionDataString = "";
int usbCommandVal = 0;
boolean USBcommandExecuted = true;
String usbCommand = "";
boolean myflag = false;
char cr = 13; //carriage return
String sendstring = "";
boolean nts = false; //need to send sendstring

unsigned long accelCheckTime;
const int accelAVGarraySize = 14;
int accel1avgXarray[accelAVGarraySize];
int accel1avgYarray[accelAVGarraySize];
int accel1avgZarray[accelAVGarraySize];
int accel1counter = 0;  //keeps track of where we are in the accel1 average arrays, used for determining what the average of the last 100 readings have been.
int maffa = 18;  //was 27.
int minAccelFFamplitudeX = maffa;
int minAccelFFamplitudeY = maffa;
int minAccelFFamplitudeZ = maffa;
int lastAXdifference = 0;  //what was the last accelerometer difference?
int lastAYdifference = 0;  //what was the last accelerometer difference?
int lastAZdifference = 0;  //what was the last accelerometer difference?
int accelCheckDelay = 16; //milliseconds between accel checks.

unsigned long lastcmdtime = millis();
int maxcmdage = 333; //max time in milliseconds arduino will wait before switching servo outputs back to neutral

void setup() {
  Serial.begin(57600);
  servoWheel.attach(3);
  servoWheel.writeMicroseconds(wheeldefault_uS);  // wheel default.
  servoThrottle.attach(5);
  servoThrottle.writeMicroseconds(throttledefault_uS);  // throttle default.
}

void sendsendstring() {
  Serial.println(sendstring);
  sendstring = "";
  nts = false;
}
void loadsendstring(String cmd, int cmdval) {
  if(sendstring == "") {
    sendstring = sendstring + cmd + cmdval;
  } else {
    sendstring = sendstring + "&" + cmd + cmdval;
  }
}
void pc(String cmd, int cmdval) {
  Serial.print(cmd);
  Serial.println(cmdval);
}

void ffb() {
  theTime = millis();
  if(theTime > accelCheckTime) {
     int accelX = analogRead(A0);    // read accel's X value
     int accelY = analogRead(A1);    // read accel's Y value
     int accelZ = analogRead(A2);    // read accel's Z value
     
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
     
     //X
     if(aXdifference != lastAXdifference) {
       //the affXresult has changed, send to host.
       //loadsendstring("<", aXdifference); //ptb(0x3C); //print "<" to spi uart, for X
       loadsendstring("<", accelX);
       lastAXdifference = aXdifference;
       nts = true;
     }
     //Y
     if(aYdifference != lastAYdifference) {
       //the affZresult has changed, send to host.
       //loadsendstring(">", aYdifference); //ptb(0x3E); //print ">", for Y
       loadsendstring(">", accelY);
       lastAYdifference = aYdifference;
       nts = true;
     }
     //Z
     if(aZdifference != lastAZdifference) {
       //the affZresult has changed, send to host.
       //loadsendstring("^", aZdifference); //ptb(0x5E); //print "^", for Z.
       loadsendstring("^", accelZ);
       lastAZdifference = aZdifference;
       nts = true;
     }
     accelCheckTime = theTime + accelCheckDelay;
  }
}

void delegate(String cmd, int cmdval) {
  if (cmd.equals("W")) {
      servoWheel.writeMicroseconds(cmdval);  // wheel
  } else if (cmd.equals("T")) {
      servoThrottle.writeMicroseconds(cmdval);  // throttle
  } else if (cmd.equals("S")) {
      if(myflag) {
        digitalWrite(13, LOW);
        myflag = false;
      } else {
        digitalWrite(13, HIGH);
        myflag = true;
      }
  }
}
void serialListen()
{
  char arduinoSerialData; //FOR CONVERTING BYTE TO CHAR. here is stored information coming from the arduino.
  String currentChar = "";
  if(Serial.available() > 0) {
    arduinoSerialData = char(Serial.read());   //BYTE TO CHAR.
    currentChar = (String)arduinoSerialData; //incoming data equated to c.
    if(!currentChar.equals("1") && !currentChar.equals("2") && !currentChar.equals("3") && !currentChar.equals("4") && !currentChar.equals("5") && !currentChar.equals("6") && !currentChar.equals("7") && !currentChar.equals("8") && !currentChar.equals("9") && !currentChar.equals("0") && !currentChar.equals(".")) { 
      //the character is not a number, not a value to go along with a command,
      //so it is probably a command.
      if(!usbInstructionDataString.equals("")) {
        //usbCommandVal = Integer.parseInt(usbInstructionDataString);
        char charBuf[30];
        usbInstructionDataString.toCharArray(charBuf, 30);
        usbCommandVal = atoi(charBuf);
      }
      if((USBcommandExecuted == false) && (arduinoSerialData == 13)) {
        delegate(usbCommand, usbCommandVal);
        USBcommandExecuted = true;
        lastcmdtime = millis();
      }
      if((arduinoSerialData != 13) && (arduinoSerialData != 10)) {
        usbCommand = currentChar;
      }
      usbInstructionDataString = "";
    } else {
      //in this case, we're probably receiving a command value.
      //store it
      usbInstructionDataString = usbInstructionDataString + currentChar;
      USBcommandExecuted = false;
    }
  }
  
  //int cmdage = millis() - lastcmdtime;
  //if(cmdage > maxcmdage) {
  //  servoWheel.writeMicroseconds(wheeldefault_uS);  // wheel default.
  //  servoThrottle.writeMicroseconds(throttledefault_uS);  // throttle default.
  //}
}

void loop() {
  serialListen();
  ffb();
  if(nts) {
    sendsendstring();
  }
}
