#include "QSerial.h"
#include <Servo.h>
#define RANGING_SENSOR_PIN 3
#define RANGE_THRESHOLD 605
//#define  LTHRESH 940
//#define  CTHRESH 940
//#define  RTHRESH 940
#define  LTHRESH 865
#define  CTHRESH 865
#define  RTHRESH 865
#define  IRL A2
#define  IRC A0
#define  IRR A1
#define txpin -1
#define rxpin 11
#define GRIP_SENSE A5
#define GRIP_THRESH 1000
#define GRIP_PIN 8
#define TILT_PIN 9
#define PAN_PIN 10
QSerial myIRserial;
Servo gripServo;
Servo tiltServo;
Servo panServo;
float LeftIR,MiddleIR,RightIR;
int MLeft = 4;
int ELeft = 5;
int ERight = 6; 
int MRight = 7;
int leftPin = 0;
int centrePin = 0;
int rightPin = 0;
int bummper_left = 2;
int bummper_right = 3;
int start = 1;
void setup() {
  Serial.begin(9600);
  pinMode(rxpin, INPUT);
  myIRserial.attach(rxpin,txpin);
  int path[2] = {1,2};
  pinMode (A0, INPUT);
  pinMode (A1, INPUT);
  pinMode (A2, INPUT);
  pinMode(MLeft, OUTPUT);
  pinMode(ELeft, OUTPUT);
  pinMode(ELeft, OUTPUT);
  pinMode(MLeft, OUTPUT);
  pinMode(bummper_left, INPUT);
  pinMode(bummper_right, INPUT);
    gripServo.attach(GRIP_PIN);
  tiltServo.attach(TILT_PIN); 
  panServo.attach(PAN_PIN);

  
}

void loop() {
   int path[15];
   int pLength;
   startPos();
   while(start){

     if(1){
       path[0] = 0;
       path[1] = 0;
       path[2] = 0;
       path[3] = 0;
       path[4] = 0;
       path[5] = 0;
       path[6] = 0;
       path[7] = 0;
       path[8] = 0;
       path[9] = 0;
       path[10] = 0;

       path[11] = 1;
       path[12] = 0;
       path[13] = 0;
       path[14] = 2;
       pLength = 15;
       start = 0;
     }
//     if(Read character '1'){
//       start = 0;
//     }
//     if(Read character '2'){
//       start = 0;
//     }
   }
  int traveled = 0;
  int LeftSpeed = 128;
  int RightSpeed = 128;
  while(traveled <= pLength){
    LeftSpeed = 128;
    RightSpeed = 128;
      leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
    //If at an intersection perform task based on path
    if(checkIntersection()){
      if(traveled < pLength){
        pathFollow(path, traveled);
        traveled++;
      }
    }
    if (leftPin > LTHRESH && centrePin < CTHRESH){
      RightSpeed += 20;
      LeftSpeed -= 20;
    }else if (rightPin > RTHRESH && centrePin < CTHRESH){
      LeftSpeed += 20;
      RightSpeed -=20;
    }
    analogWrite(ELeft, LeftSpeed);
    analogWrite(ERight, RightSpeed);
    digitalWrite(MLeft, 1);
    digitalWrite(MRight, 1);
    if(wallStop()){
      if(!checkGrip()){
        reverse();
        delay(325);
        stopRobot();
        pickUp();
        turnAround();
      }else{
        stopRobot();
        dropOff();
        turnAround();
      }
    }
  }
}

void pathFollow(int path[], int intersection){
  Serial.println(path[intersection]);
  if(path[intersection] == 0){
    delay(500);
  }else if(path[intersection] == 1){
    turnLeft();
  }else if(path[intersection] == 2){
    turnRight();
  }else if(path[intersection] == 3){
//    stopRobot();
  }
  
}

void turnLeft(){
  analogWrite(ELeft, 128);
  analogWrite(ERight, 128);
  digitalWrite(MLeft, 1);
  digitalWrite(MRight, 1);
  delay(500);
  digitalWrite(MLeft, 0);
  digitalWrite(MRight, 1);
  delay(500);
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  while(centrePin <= CTHRESH){
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
    delay(1);
  }
  analogWrite(ELeft, 0);
  analogWrite(ERight, 0);
}

void turnRight(){
  analogWrite(ELeft, 128);
  analogWrite(ERight, 128);
  digitalWrite(MLeft, 1);
  digitalWrite(MRight, 1);
  delay(500);
  digitalWrite(MLeft, 1);
  digitalWrite(MRight, 0);
  delay(500);
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  while(centrePin <= CTHRESH){
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
    delay(1);
  }
  analogWrite(ELeft, 0);
  analogWrite(ERight, 0);
}
int checkIntersection(){
    leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  if ((leftPin >= LTHRESH)&&(centrePin >= CTHRESH)&&(rightPin >= RTHRESH)){
    Serial.println("intersection");
    return 1;
  }else{
    return 0;
  }
}

char beaconReceiver(){
  char val;
  while(1){
    val = myIRserial.receive(200);
    Serial.println(val);
    if(val == '0'){
      return val;
    }
    else if(val == '1'){
      return val;
    }
    else if(val == '2'){
      return val;
    }
    delay(100);
  }
}

void turnAround(){
  reverse();
  delay(300);
  digitalWrite(MLeft, 1);
  digitalWrite(MRight, 0);
    analogWrite(ELeft, 128);
  analogWrite(ERight, 128);
  delay(500);
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  while(centrePin <= CTHRESH){
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  //  delay(1);
  }
  analogWrite(ELeft, 0);
  analogWrite(ERight, 0);
}
void startPos(){ //debugged n fine 
  //starting position of the arm, pan - 90, vertical - 160, fully open - 40 
  panServo.write(96);
  tiltServo.write(160);
  gripServo.write(40);
  }

 void walkingPos(){ //vertical posn = 160 for us  debugged n fine
  Serial.println("walking around now");
  panServo.write(90);
  tiltServo.write(160);
 }

 
int rangingStop(){
  int range = analogRead(RANGING_SENSOR_PIN);
  Serial.println(range);
  if(range > RANGE_THRESHOLD){
    return 1;
  }
  else
    return 0;
}

int wallStop(){
  int bl = digitalRead(bummper_left);
  int br = digitalRead(bummper_right);
  if(br && bl){
    return 1;
  }
  return 0;
}

void stopRobot(){
   analogWrite(ELeft, 0);
    analogWrite(ERight, 0);
}

void closeGripper(){ //debugged n fine 
 int press = analogRead(GRIP_SENSE); 
 int i = 40;
   while(press < GRIP_THRESH){ //if closed on nothing
      press = analogRead(GRIP_SENSE);
      Serial.println(press);
      gripServo.write(i);
      delay(50);
      if(i == 179){
        break;
      }
      i++;
   }
   
 if(press >= GRIP_THRESH){ //if closing on something
   Serial.println("closed");
    int angle = gripServo.read(); //angle gripper is at
   Serial.println(angle);
  } 
delay(1000);
}

void dropOff(){ //have ball, bringing home
  gripServo.write(40);
  delay(100);
  Serial.println("Dropping off");
  tiltServo.write(75);
  delay(1000); //this delay is necessary
  walkingPos();
}

void pickUp() { //getting from wall debugged n fine
  Serial.println("Picking up");
  startPos();
  tiltServo.write(75); //horizontal = 75 for us
  closeGripper();
  walkingPos(); 
}

void reverse(){
  digitalWrite(MLeft, 0);
  digitalWrite(MRight, 0);
  analogWrite(ELeft, 128);
  analogWrite(ERight, 128);
}

int checkGrip(){
  int press = analogRead(GRIP_SENSE); 
  if(press > GRIP_THRESH){
    return 1;
  }
  return 0;
}
    
