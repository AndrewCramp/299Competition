#include "QSerial.h"
#include <Servo.h>
#define RANGING_SENSOR_PIN 3
#define RANGE_THRESHOLD 605
//#define  LTHRESH 940
//#define  CTHRESH 940
//#define  RTHRESH 940
#define  LTHRESH 915
#define  CTHRESH 915
#define  RTHRESH 915
#define  IRL A2
#define  IRC A0
#define  IRR A1
#define txpin -1
#define rxpin 11
#define GRIP_SENSE 5
#define GRIP_THRESH 600
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
int returning = 0;
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
   int path[43];
   int pLength;
startPos();
   while(start){

//     if(1){
       path[0] = 0;
       path[1] = 0;
       path[2] = 0;
       path[3] = 0;
       path[4] = 0;
       
       path[5] = 0;
       path[6] = 0;
       path[7] = 0;
       path[8] = 0;
       

       path[9] = 1;
       path[10] = 0;
       path[11] = 2;

       path[12] = 0;
       path[13] = 0;
       path[14] = 2;
       path[15] = 0;
       path[16] = 0;
       path[17] = 0;
       path[18] = 0;
       path[19] = 0;
       path[20] = 1;
       path[21] = 0;
       path[22] = 0;

       path[23] = 0;
       path[24] = 0;
       path[25] = 0;
       path[26] = 1;
       path[27] = 0;
       path[28] = 2;
       path[29] = 0;
       path[30] = 0;
       path[31] = 0;

       path[32] = 0;
       path[33] = 0;
       path[34] = 0;
       path[35] = 0;
       path[36] = 1;
       path[37] = 0;
       path[38] = 2;
       path[39] = 0;
       path[40] = 0;
       path[41] = 0;
       path[42] = 0;      
       
       pLength = 43;
       start = 0;
//     }
  //   if(1){
      //1 is Left-Hand Turn, 2 is Right-Hand Turn, 0 is go straight
      //Proceed to (1)
       path[0] = 0;
       path[1] = 0;
       path[2] = 0;
       path[3] = 0;
       path[4] = 0;
       //Hit (1), Turn Around (ignore next intersection)
       path[5] = 0;
       path[6] = 0;
       path[7] = 0;
       path[8] = 0;
       //Hit Goal, Turn Around
       //Proceed to (2)
       path[9] = 0;
       path[10] = 1;
       path[11] = 0;
       path[12] = 0;
       //Hit (2), Turn Around (ignore next intersection) 
       path[13] = 0;
       path[14] = 2;
       path[15] = 0;
       //Hit Goal, Turn Around
       //Proceed to (4), to avoid collision with Left Robot on Path (3)
       path[16] = 0;
       path[17] = 0;
       path[18] = 0;
       path[19] = 0;
       path[20] = 1;
       path[21] = 0;
       path[22] = 2;
       //Hit (4), Turn Around (ignore next intersection o o f)
       path[23] = 1;
       path[24] = 0;
       path[25] = 2;
       path[26] = 0;
       path[27] = 0;
       path[28] = 0;
       //Hit Goal Turn round
       //Proceed to (3) 
       path[29] = 0;
       path[30] = 2;
       path[31] = 0;
       path[32] = 0;
       //Hit (3), Turn Around (ignore next intersection)
       path[33] = 0;
       path[34] = 1;
       path[35] = 0;
       //Hit Goal, Turn Around
       //Proceed to (5)
       path[36] = 0;
       path[37] = 0;
       path[38] = 0;
       path[39] = 0;
       path[40] = 2;
       path[41] = 0;
       path[42] = 0;
       //Hit (5), Turn Around (ignore next intersection)
       path[43] = 0;
       path[44] = 1;
       path[45] = 0;
       path[46] = 0;
       path[47] = 0;
       path[48] = 0;
       //Hit Goal
       pLength = 49;
       start = 0;
    // }
    if(1){
      //1 is Left-Hand Turn, 2 is Right-Hand Turn, 0 is go straight
      //Proceed to (1)
       path[0] = 0;
       path[1] = 0;
       path[2] = 0;
       path[3] = 0;
       path[4] = 0;
       //Hit (1), Turn Around (ignore next intersection)
       path[5] = 0;
       path[6] = 0;
       path[7] = 0;
       path[8] = 0;
       //Hit Goal, Turn Around
       //Proceed to (2)
       path[9] = 0;
       path[10] = 0;
       path[11] = 1;
       path[12] = 0;
       path[13] = 0;
       path[14] = 0;
       //Hit (2), Turn Around (ignore next intersection) 
       path[15] = 0;
       path[16] = 0;
       path[17] = 2;
       path[18] = 0;
       path[19] = 0;
       //Hit Goal, Turn Around
       //Proceed to (3)       
       path[20] = 2;
       path[21] = 0;
       //Hit (3), Turn Around (ignore next intersection)
       path[22] = 1;
       //Hit Goal Turn round
       //Proceed to (4)
       path[23] = 0;
       path[24] = 0;
       path[25] = 0;
       path[26] = 0;
       path[27] = 2;
       path[28] = 1;
       //Hit (4), Turn Around (ignore next intersection)
       path[29] = 2;       
       path[30] = 1;
       path[31] = 0;
       path[32] = 0;
       path[33] = 0;
       //Hit Goal, Turn Around
       //Proceed to (5)
       path[34] = 0;
       path[35] = 0;
       path[36] = 0;
       path[37] = 2;
       path[38] = 0;
       //Hit (5), Turn Around (ignore next intersection)
       path[39] = 1;
       path[40] = 0;
       path[41] = 0;
       path[42] = 0;
       //Hit Goal
       pLength = 43;
       start = 0;
     start = 0;
     }
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
      RightSpeed += 30;
      LeftSpeed -= 30;
    }else if (rightPin > RTHRESH && centrePin < CTHRESH){
      LeftSpeed += 30;
      RightSpeed -=30;
    }
    analogWrite(ELeft, LeftSpeed);
    analogWrite(ERight, RightSpeed);
    digitalWrite(MLeft, 1);
    digitalWrite(MRight, 1);
    delay(10);
    if(wallStop()){
      if(!returning){
        reverse();
        delay(325);
        stopRobot();
        centerPos();
        pickUp();
        turnAround(2);
      }else{
        stopRobot();
        dropOff();
        turnAround(1);
      }
    }
  }
}

void pathFollow(int path[], int intersection){

  if(path[intersection] == 0){
    delay(200);
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

    return 1;
  }else{
    return 0;
  }
}

char beaconReceiver(){
  char val;
  while(1){
    val = myIRserial.receive(200);

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

//void turnAround(){
//  reverse();
//  delay(800);
//  digitalWrite(MLeft, 1);
//  digitalWrite(MRight, 0);
//    analogWrite(ELeft, 128);
//  analogWrite(ERight, 128);
//  delay(1400);
//  leftPin = analogRead(IRL);  
//  centrePin = analogRead(IRC);
//  rightPin = analogRead(IRR);
//  while(centrePin <= 900){
//    Serial.println(centrePin);
//  leftPin = analogRead(IRL);  
//  centrePin = analogRead(IRC);
//  rightPin = analogRead(IRR);
//    delay(1);
//  }
//  analogWrite(ELeft, 0);
//  analogWrite(ERight, 0);
//}
void startPos(){ //debugged n fine 
  //starting position of the arm, pan - 90, vertical - 160, fully open - 40 
  panServo.write(96);
  tiltServo.write(160);
  gripServo.write(40);
  }

void turnAround(int number){
  reverse();
  delay(800);
  digitalWrite(MLeft, 1);
  digitalWrite(MRight, 0);
    analogWrite(ELeft, 128);
  analogWrite(ERight, 128);
  delay(200);
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  int count = 0;
  while(count < number){
    Serial.println(centrePin);
  leftPin = analogRead(IRL);  
  centrePin = analogRead(IRC);
  rightPin = analogRead(IRR);
  if(centrePin > 900){
    count++;
    if(count <= number-1){
    delay(200);
    }
  }
  }
  analogWrite(ELeft, 0);
  analogWrite(ERight, 0);
}


 void walkingPos(){ //vertical posn = 160 for us  debugged n fine

  panServo.write(90);
  tiltServo.write(160);
 }

 
int rangingStop(){
  int range = analogRead(RANGING_SENSOR_PIN);

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
      gripServo.write(i);
      delay(50);
      if(i > 130){
        gripServo.write(40);
        break;
      }
      i++;
   }
   
 if(press >= GRIP_THRESH){ //if closing on something
    int angle = gripServo.read(); //angle gripper is at
  } 
delay(1000);
}

void dropOff(){ //have ball, bringing home
  gripServo.write(40);
  delay(100);
  tiltServo.write(75);
  delay(1000); //this delay is necessary
  walkingPos();
  returning = 0;
}

void pickUp() { //getting from wall debugged n fine
  startPos();
  tiltServo.write(75); //horizontal = 75 for us
  closeGripper();
  walkingPos(); 
  returning = 1;
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

void centerPos(){
  gripServo.write(179);
  tiltServo.write(85);
  panServo.write(50);
  for (int i=50; i<85; i++){
      panServo.write(i);
      delay(50);
  }
  delay (1000); 
  tiltServo.write(160); 
  delay(1000); 
  
  gripServo.write(179);
  panServo.write(130);
  delay(1000); 
  tiltServo.write(85);
  for (int i=130; i>105; i--){
      panServo.write(i);
      delay(50);
  }
  delay(1000);
  gripServo.write(40);
  tiltServo.write(160);
  delay(500);
  panServo.write(90);
 }
    
