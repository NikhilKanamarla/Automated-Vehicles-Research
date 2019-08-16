#include <FastGPIO.h>
#include <LSM303.h>
#include <PololuBuzzer.h>
#include <PololuHD44780.h>
#include <USBPause.h>
#include <Zumo32U4.h>
#include <Zumo32U4Buttons.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4Motors.h>

Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4Encoders encoders;

// Variables
const char connectedmusic[] PROGMEM = "! O5 L16 cde";
const int numReadings = 25;
int readIndex;

int rightEncoderReadings[numReadings];// the readings from the right encoder feedback
int leftEncoderReadings[numReadings];
int rightCounts;
int leftCounts;
int rightCountsChange;
int leftCountsChange;
int rightCountsTotal;
int leftCountsTotal;
String rightVelocityInput;
String leftVelocityInput;
double rightVelocityTarget;
double leftVelocityTarget;
double rightCountsTarget; // from Caili's code, used for PID
double leftCountsTarget;
double rightCountsPerSecond;
double leftCountsPerSecond;
double rightError;
double leftError;
double rightErrorTotal;
double leftErrorTotal;

int timeChange;
int timeLast;
double timeTotal; //WATCH OUT FOR OVERFLOW
int timeReadings[numReadings];      // the readings from the right encoder feedback

// Note free run speed is 400 RPM @6V = 400*0.032*3.14159/60 = 0.67 m/s max velocity @ 6V
// https://www.pololu.com/product/3126 (75:1 gear ratio, actually its 75.81:1 - pesky gears)
//Note 909.7 Counts per revolution @ 75.81:1 https://www.pol,,,olu.com/docs/0J63/3.4
double wheelDia = 0.032; //units are meters (wheel diameter is 32 mm)
double pi = 3.14159;
double countsPerRev = 909.7;

double kp = .5;
double ki = 1; //0.001;
double rightInput = 200;
double rightInputLast = 0;
double leftInput = 200;
double leftInputLast = 0;
uint16_t batteryLevel;

void setup() {
  Serial.begin(115200);
  buzzer.playFromProgramSpace(connectedmusic);
  rightCounts = encoders.getCountsRight();
  leftCounts = encoders.getCountsLeft();
  rightCountsChange = 0;
  leftCountsChange = 0;
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    rightEncoderReadings[thisReading] = 0;
    leftEncoderReadings[thisReading] = 0;
  }
  readIndex = 0;
  timeLast = millis(); //initialize time counter
  rightCounts = encoders.getCountsRight(); //initialize encoder counter
  leftCounts = encoders.getCountsLeft();
}

void loop() {
  
  if(Serial.available()>0) { //here we get the target encoder counts/second from Caili's code
    rightVelocityInput = Serial.readStringUntil(','); 
    rightVelocityTarget = rightVelocityInput.toDouble();
    leftVelocityInput = Serial.readStringUntil(';'); 
    leftVelocityTarget = leftVelocityInput.toDouble();
    batteryLevel = readBatteryMillivolts();
  }
  Serial.println(batteryLevel);
  //rightVelocityTarget = 0.5;// units are m/s (0.032*pi=0.10053088 would be 1 rev/second)
  ///leftVelocityTarget = .5;
  rightCountsTarget = rightVelocityTarget*countsPerRev/(wheelDia*pi); // counts per second, units cancel
  leftCountsTarget = leftVelocityTarget*countsPerRev/(wheelDia*pi);
  
  timeChange = millis() - timeLast;
  timeLast = timeChange + timeLast; //to avoid calling millis() again (waste of time and wrong value)
  timeTotal = timeTotal - timeReadings[readIndex];
  timeReadings[readIndex] = timeChange;
  timeTotal = timeTotal + timeReadings[readIndex];
  
  rightCountsChange = encoders.getCountsRight() - rightCounts;
  leftCountsChange = encoders.getCountsLeft() - leftCounts;
  rightCounts = rightCountsChange + rightCounts; //avoids fulling the encoder count a second time
  leftCounts = leftCountsChange + leftCounts;
  rightCountsTotal = rightCountsTotal - rightEncoderReadings[readIndex];
  leftCountsTotal = leftCountsTotal - leftEncoderReadings[readIndex];
  rightEncoderReadings[readIndex] = rightCountsChange;
  leftEncoderReadings[readIndex] = leftCountsChange;
  rightCountsTotal = rightCountsTotal + rightEncoderReadings[readIndex];
  leftCountsTotal = leftCountsTotal + leftEncoderReadings[readIndex];

  rightCountsPerSecond = rightCountsTotal/timeTotal * 1000; //convert from millis to seconds
  leftCountsPerSecond = leftCountsTotal/timeTotal * 1000;
  rightError = rightCountsTarget - rightCountsPerSecond;
  leftError = leftCountsTarget - leftCountsPerSecond;
  //rightErrorTotal += rightError; //RETURNS NAN FOR SOME REASON??
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {readIndex = 0;}
  rightInput = constrain(kp*rightError,0,400); //ki*rightErrorTotal
  leftInput = constrain(kp*leftError,0,400);
  //////////////////////////////////////////////////////////////////
  motors.setSpeeds(leftInput,rightInput);
}
