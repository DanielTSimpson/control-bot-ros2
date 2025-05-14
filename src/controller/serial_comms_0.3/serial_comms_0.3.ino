/*
   Used to get and apply serial data from a Jetson Orin Nano for the speeds and directions of four motors in the form [DELIMITER, D1, M1, D2, M3, D3, M3, D4, M4, DELIMITER] where each element is printedonce per line of serial data read, where DX is the direction (0, 1) and MX is the motor speed (0-254), where X is the motor ID (1, 2, 3, 4)
   Some code borrowed from: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3?_gl=1*1g0j69o*_up*MQ..*_ga*MTAwMjg5MjM4LjE3MzkzMjMzNzg.*_ga_NEXN8H46L5*MTczOTMyMzM3Ny4xLjAuMTczOTMyMzM3Ny4wLjAuMjEzMjAwMDQyNA..

*/

#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

const byte numChars = 8;
bool gotData = false;
uint8_t mtrDir = FORWARD;
unsigned long lastCmdTime = 0;
int failsafeDelay = 100;

void setup() {
  Serial.begin(115200);
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  delay(1000);
}

void loop() {
  int mtrData[numChars];
  getInstructions(mtrData);
  if (gotData){    
    setMotors(mtrData);
    lastCmdTime = millis();
  }  
  else if (millis() - lastCmdTime > failsafeDelay){
    memset(mtrData, 0, sizeof(mtrData));
    setMotors(mtrData);
  }
  gotData = false;
}

void getInstructions(int receivedBytes[numChars]) {
  byte delimByte = 255;
  byte msg;
  static byte i = 0;
  static bool receivingData = false;
  
  while (Serial.available() > 0) {
    msg = Serial.read();
    if (msg == delimByte && receivingData == false) {
      receivingData = true;
    } else if (msg == delimByte  && receivingData == true) {
      receivingData = false;
    } else if (msg != delimByte && receivingData == true) {
      receivedBytes[i] = msg;
      i++;
    }
    if (i >= numChars) {
      i = 0;
      gotData = true;
      return receivedBytes;
    }
  }
}

void setMotors(int mtrCmds[numChars]) {
  for (int i = 0; i < numChars; i++){
    if (i%2 == 0){
      if (mtrCmds[i] == 0){
        mtrDir = FORWARD;
      } else {
        mtrDir = BACKWARD;
      }
    } else {  
        if (i == 1) {
          myMotor1->run(mtrDir);
          myMotor1->setSpeed(mtrCmds[i]);
        } else if (i == 3) {
          myMotor2->run(mtrDir);
          myMotor2->setSpeed(mtrCmds[i]);
        } else if (i == 5) {
          myMotor3->run(mtrDir);
          myMotor3->setSpeed(mtrCmds[i]);
        } else if (i == 7) {
          myMotor4->run(mtrDir);
          myMotor4->setSpeed(mtrCmds[i]);
        }
    }
  }
}
