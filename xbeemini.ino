
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
int maffa = 15;
int lastAX = 341;  
int lastAY = 353;  
int lastAZ = 406;  
int accelCheckDelay = 20; //milliseconds between accel checks.

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

void watchaccel() {
  theTime = millis();
  if(theTime > accelCheckTime) {
     int accelX = analogRead(A0);    // read accel's X value
     int accelY = analogRead(A1);    // read accel's Y value
     int accelZ = analogRead(A2);    // read accel's Z value
     
     //determine if X reading is out of range
     int accelUnder = lastAX - maffa;
     int accelOver = lastAX + maffa;
     if(accelX <= accelUnder || accelX >= accelOver) {
       loadsendstring("x", accelX);
       lastAX = accelX;
       nts = true;
     }
     //determine if Y reading is out of range
     accelUnder = lastAY - maffa;
     accelOver = lastAY + maffa;
     if(accelY <= accelUnder || accelY >= accelOver) {
       loadsendstring("y", accelY);
       lastAY = accelY;
       nts = true;
     }
     //determine if Z reading is out of range
     accelUnder = lastAZ - maffa;
     accelOver = lastAZ + maffa;
     if(accelZ <= accelUnder || accelZ >= accelOver) {
       loadsendstring("z", accelZ);
       lastAZ = accelZ;
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
  watchaccel();
  if(nts) {
    sendsendstring();
  }
}
