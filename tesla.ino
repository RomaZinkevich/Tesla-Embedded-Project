#include <LiquidCrystal.h>
#include <Wire.h>

#define CMPS14_address 0x60 // I2C slave address for CMPS compass module 
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

#include "LIDARLite_v4LED.h"
#include <EEPROM.h>
#include "DFRobot_TCS34725.h"

LIDARLite_v4LED myLIDAR;


LiquidCrystal lcd(37, 36, 35, 34, 33, 32);

const int joystickXPin = A8; // X-axis pin
const int joystickYPin = A9; // Y-axis pin
const int joystickButtonPin = 19; //Joystick's button pin
const int encoderPinRight = 3; //Pin for right wheel
const int encoderPinLeft = 2; //Pin for left wheel

boolean buttonPressed = false; //Flag to determine if button is pressed
volatile long pulseCount = 0;
volatile unsigned long lastDebounceTime = 0; // The last time the button was pressed
const unsigned long debounceDelay = 50; 
int offset=220;//offset for compass

int totaldistance=0;
float wallDistance=0;
bool isFollowing=false;

float volume=0;
float area=0;

int RED, GREEN, BLUE=0;


DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS,TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Wire.begin(); //Setup for compass

  //Setup for wheels
  pinMode(encoderPinRight, INPUT);
  pinMode(encoderPinLeft, INPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinRight), updatePulseCount, RISING);

  lcd.begin(20, 4);

  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.print("LESGOOOOO");

  while(!tcs.begin())
  {
    Serial.println("No TCS34725 found ... check your connections");
    delay(1000);
  }

}

void loop() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
  colorSensing();   
  sendToESP();
  webMode();
  printData();//Printing all required data to LCD
}

void colorSensing(){
  uint16_t clear, red, green, blue;
  tcs.getRGBC(&red, &green, &blue, &clear);
  // turn off LED
  tcs.lock();  

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red;
  g = green; 
  b = blue;
  r = (float)red / sum * 255;
  g = (float)green / sum * 255;
  b = (float)blue / sum * 255;
  RED=r;
  GREEN=g;
  BLUE=b;
}

void sendToESP(){
  int initDegrees;
  Wire.beginTransmission(CMPS14_address);    
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CMPS14_address, 2, true); 
  if (Wire.available() >= 2) { 
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int heading = (highByte << 8) + lowByte;
    initDegrees = (heading / 10 + offset) % 360; 
  }
  String lid = "Lid="+String(wallDistance);
  if (wallDistance<=10) lid = "Lid=Warning!";
  String com = "Com="+String(initDegrees);
  String color = "Col="+String(RED)+";"+String(GREEN)+";"+String(BLUE);
  Serial3.println(lid);
  printData();
  delay(250);
  Serial3.println(color);
  printData();
  delay(250);
  Serial3.println(com);
}

void printData(){
  lcd.clear();
  float readData;
  // for (int i=1; i<3; i++){
  //   readData= EEPROM.read(i);
  //   if (i==2)
  //     Serial.print("Wood: ");
  //   else
  //     Serial.print("Lid: ");
  //   Serial.print(String(readData/1000, 5)); 
  //   Serial.println();
  // }
  lcd.setCursor(0,0);
  printDistance();
  printDriven();
  if (volume!=0) {
    printMeasures();
  }
  delay(200);
}

void printDistance(){
  wallDistance = myLIDAR.getDistance();  
  lcd.setCursor(0,0);
  lcd.print("Dist:");
  lcd.print(wallDistance);
  lcd.print(" cm");
}

void printDriven(){
  lcd.setCursor(0,1);
  lcd.print("Driven:");
  lcd.print(totaldistance);
  lcd.print(" cm");
}

void printMeasures(){
  lcd.setCursor(0,2);
  lcd.print("Volume:");
  lcd.print(volume);
  lcd.print(" m^3");
  lcd.setCursor(0,3);
  lcd.print("Area:");
  lcd.print(area);
  lcd.print(" m^2");
}

void webMode(){//Mode when car can be driven from webpage
  bool flag = false;
  String message;
  if (Serial.available() > 0) {//If some message arrived
    message = Serial.readStringUntil('\n');
    flag = true;
  }
  else if (Serial3.available() > 0) { 
    message = Serial3.readStringUntil('\n');
    flag = true;
  }
  if (flag==true){
    Serial.println(message);
    if (message.indexOf("Move")==0){
      int pos_s = message.indexOf(":");
      int stat = message.substring(pos_s + 1).toInt();
      if (stat >= 0) message = "Forward:"+String(stat);
      else message = "Backward:"+String(abs(stat));
    }
    int pos_s = message.indexOf(":");
    //Gets index of Command (Can be [Move, Turn, DIr])
    int forwardcommand = message.indexOf("Forward");
    int backwardcommand = message.indexOf("Backward");
    int followcommand = message.indexOf("Follow");
    int squarePotcommand = message.indexOf("SquarePot");
    int squarecommand = message.indexOf("Square");
    int measurecommand = message.indexOf("Measure");
    int pathcommand = message.indexOf("Path");
    int compcommand = message.indexOf("Comp");
    int calcommand = message.indexOf("Cal");
    int turncommand = message.indexOf("Turn");
    int mazecommand = message.indexOf("Maze");
    if(pos_s > -1) {//If ":" was in the message then we have right command
      if (forwardcommand==0){
        int stat = message.substring(pos_s + 1).toInt();//Gets number from the message
        if (stat > 0 && stat < 21) {//Can be moved 20 forward and 20 back
          Serial.print("Drive: ");
          Serial.println(stat);
          drive(0, stat);
          isFollowing=false;
        } else {
          Serial.println("Put proper number 0 - 20");
        }
      }
      else if (turncommand==0){
        int stat = message.substring(pos_s + 1).toInt();
        turn2(0,stat);
      }
      else if (backwardcommand==0){
        int stat = message.substring(pos_s + 1).toInt();//Gets number from the message
        if (stat > 0 && stat < 21) {//Can be moved 20 forward and 20 back
          Serial.print("Drive: ");
          Serial.println(stat*-1);
          drive(0, stat*-1);
          isFollowing=false;
        } else {
          Serial.println("Put proper number 0 - 20");
        }
        
      }
      else if (followcommand==0){
        isFollowing=true;
      }
      else if (squarePotcommand==0) {
        square(readAnalog());
      }
      else if (squarecommand==0) {
        int stat = message.substring(pos_s + 1).toInt();
        square(stat);
      }
      else if (measurecommand==0) {
        measureRoom();
      }
      else if (pathcommand==0) {
        path();
      }
      else if (compcommand==0){
        comp();
      }
      else if (calcommand==0){
        int stat = message.substring(pos_s + 1).toInt();
        calibrate(stat);
      }
      else if (mazecommand==0){
        maze();
      }
    }
  }
  else if (isFollowing){
    follow();
  }
}

String domColor(){
  if (RED>GREEN && RED>BLUE)
    return "red";
  else if (GREEN > RED && GREEN > BLUE)
    return "green";
  else return "blue";
}

void maze(){
  String color=domColor();
  Serial.println(color);
  while(color!="green"){
    colorSensing();
    color=domColor();
    Serial.print("Blue:");
    Serial.println(BLUE);
    mazeDrive();
    if (color=="red") turn2(0,-10);
    else if (color=="blue" && BLUE>97) turn2(0,10);
  }
}

void calibrate(int memoryNum){
  float pulses=0.00;
  pulseCount=0;
  float dist=0.00;
  for (int i=0; i<5; i++){
    drive(0, 20);
    pulses+=pulseCount;
    drive(0, -20);
    pulseCount=0;
  }
  dist = ((100/pulses)*1000);
  EEPROM.update(memoryNum, dist);
}

void path(){
  followN(25);
  turn2(0, -85);
  followN(20);
  turn2(0, -87);
  followN(25);
  turn2(0, -82);
  followN(20);
  turn2(0, -80);
}

void comp(){
  turn2(0, 90);
  followN(70);
  turn2(0, -90);
  followN(25);
  turn2(0, -90);
  followN(51);
}

void square(int dist){
  drive(0, dist);
  turn(0, 90);
  drive(0, dist);
}

int readAnalog(){
  int value = analogRead(A5); 
  int mappedValue = map(value, 0, 1023, 2, 20);
  return mappedValue;
}

void follow(){
  drive(0, wallDistance-readAnalog());
}

void followN(int n){
  drive(0, wallDistance-n);
}

void measureRoom(){
  float x1 = myLIDAR.getDistance();  
  turn2(0,90);
  float y1 = myLIDAR.getDistance(); 
  turn2(0,80);
  float x2 = myLIDAR.getDistance(); 
  turn2(0,-270);
  float y2 = myLIDAR.getDistance(); 
  delay(5000);
  float h = myLIDAR.getDistance() + 20; 
  lcd.setCursor(0,2);
  lcd.print(x1+x2);
  Serial.println(x1+x2);
  Serial.println(y1+y2);
  Serial.println(h);
  area = (x1+x2)*(y1+y2)/10000;
  volume = area * h / 100;
}

void turn2(int encoderValue, int extraDegrees) {
  int fin, initDegrees;
  //calculates position of the car before rotating
  //also calculates position it supposed to be in
  Wire.beginTransmission(CMPS14_address);    
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CMPS14_address, 2, true); 
  if (Wire.available() >= 2) { 
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int heading = (highByte << 8) + lowByte;
    int initDegrees = (heading / 10 + offset) % 360; 
    fin = initDegrees + extraDegrees;
  }

  //while true loop that breaks if car reached wanted position
  while (true){
    printData();
    //calculating position of the car
    Wire.beginTransmission(CMPS14_address);    
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 2, true); 
    if (Wire.available() >= 2) { 
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      int heading = (highByte << 8) + lowByte;
      int degrees = (heading / 10 + offset) % 360; 
      // Clockwise rotation
      if (extraDegrees>0){//Looping "position", so it goes 0-359 and 360-359
        if ((degrees < fin) && (fin-degrees) < 360){
          Serial.println(degrees);
          digitalWrite(Motor_R_dir_pin, 1);
          digitalWrite(Motor_L_dir_pin, 0);
          analogWrite(Motor_L_pwm_pin, 100);
          analogWrite(Motor_R_pwm_pin, 100);
        }
        else if ((fin-degrees) > 360) {
          fin-=360;
        }
        else {//If it reached required degrees car stops
          stopMovement();
          break;
        }
      }
      //Counter-clockwise rotation
      else if (extraDegrees<0){
        if ((degrees > fin) && (degrees-fin) < 360){
          Serial.println(degrees);
          Serial.println(fin);
          digitalWrite(Motor_R_dir_pin, 0);
          digitalWrite(Motor_L_dir_pin, 1);
          analogWrite(Motor_R_pwm_pin, 100);
          analogWrite(Motor_L_pwm_pin, 100);
        }
        else if ((degrees-fin) > 360) {
          fin+=360;
        }
        else {
          stopMovement();
          break;
        }
      }
      
    }
  }
}

void turn(int encoderValue, int extraDegrees) {//"Turn" command
  int fin, initDegrees;
  //calculates position of the car before rotating
  //also calculates position it supposed to be in
  Wire.beginTransmission(CMPS14_address);    
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CMPS14_address, 2, true); 
  if (Wire.available() >= 2) { 
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int heading = (highByte << 8) + lowByte;
    int initDegrees = (heading / 10 + offset) % 360; 
    fin = initDegrees + extraDegrees;
  }

  //while true loop that breaks if car reached wanted position
  while (true){
    printData();
    //calculating position of the car
    Wire.beginTransmission(CMPS14_address);    
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 2, true); 
    if (Wire.available() >= 2) { 
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      int heading = (highByte << 8) + lowByte;
      int degrees = (heading / 10 + offset) % 360; 
      // Clockwise rotation
      if (extraDegrees>0){//Looping "position", so it goes 0-359 and 360-359
        if ((degrees < fin) && (fin-degrees) < 360){
          Serial.println(degrees);
          digitalWrite(Motor_R_dir_pin, 0);
          digitalWrite(Motor_L_dir_pin, 0);
          analogWrite(Motor_L_pwm_pin, 150);
          analogWrite(Motor_R_pwm_pin, 0);
        }
        else if ((fin-degrees) > 360) {
          fin-=360;
        }
        else {//If it reached required degrees car stops
          stopMovement();
          break;
        }
      }
      //Counter-clockwise rotation
      else if (extraDegrees<0){
        if ((degrees > fin) && (degrees-fin) < 360){
          Serial.println(degrees);
          Serial.println(fin);
          digitalWrite(Motor_R_dir_pin, 0);
          digitalWrite(Motor_L_dir_pin, 0);
          analogWrite(Motor_R_pwm_pin, 150);
          analogWrite(Motor_L_pwm_pin, 0);
        }
        else if ((degrees-fin) > 360) {
          fin+=360;
        }
        else {
          stopMovement();
          break;
        }
      }
      
    }
  }
  
}

void mazeDrive() {
  digitalWrite(Motor_R_dir_pin, 0);
  digitalWrite(Motor_L_dir_pin, 0);
  analogWrite(Motor_L_pwm_pin, 100);
  analogWrite(Motor_R_pwm_pin, 100);
  String color=domColor();
  while (color !="green" && color!="red" && BLUE<97){
    Serial.println(color);
    Serial.println(BLUE);
    colorSensing();
    color=domColor();
    analogWrite(Motor_L_pwm_pin, 100);
    analogWrite(Motor_R_pwm_pin, 100);
  }
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}

void drive(int encoderValue, int distance) {//"Move" command
  int left, right;
  int speed;
  int initPulsesDist, pulsesDist=0, initDegrees, degrees, heading;
  float initDistance = wallDistance;
  float finDistance = initDistance-distance;
  int amountOfLookingAround=0;
  totaldistance+=distance;

  Wire.beginTransmission(CMPS14_address);    
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CMPS14_address, 2, true); 
  if (Wire.available() >= 2) { 
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    heading = (highByte << 8) + lowByte;
    initDegrees = (heading / 10 + offset) % 360; 
  }

  //Determines direction according to negative or positive value of "distance"
  if (distance < 0) {
    left = 1;
    right = 1;
    distance = distance*-1;
  } else if (distance > 0) {
    left = 0;
    right = 0;
  }

  if (wallDistance<=30) speed=100;
  else speed=200;

  Serial.print("DGR:");
  Serial.println(initDegrees);
  while ((wallDistance <= 10 && left==0) && (amountOfLookingAround<6)) {
    lookaround();
    amountOfLookingAround+=1;
    Wire.beginTransmission(CMPS14_address);    
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 2, true); 
    if (Wire.available() >= 2) { 
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      heading = (highByte << 8) + lowByte;
      initDegrees = (heading / 10 + offset) % 360; 
    }
    finDistance = wallDistance-distance;
  }
  bool statement1 = (((wallDistance > finDistance) && left==0) || ((wallDistance < finDistance) && left==1));
  bool statement2 = (wallDistance > 10 || (wallDistance<=10 && left==1));
  Serial.print("DGR:");
  Serial.println(initDegrees);

  digitalWrite(Motor_R_dir_pin, right);
  digitalWrite(Motor_L_dir_pin, left);
  analogWrite(Motor_L_pwm_pin, speed);
  analogWrite(Motor_R_pwm_pin, speed);

  while (statement1 && statement2){
    Wire.beginTransmission(CMPS14_address);    
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 2, true); 
    if (Wire.available() >= 2) { 
      digitalWrite(Motor_R_dir_pin, right);
      digitalWrite(Motor_L_dir_pin, left);
      analogWrite(Motor_L_pwm_pin, speed);
      analogWrite(Motor_R_pwm_pin, speed);
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      heading = (highByte << 8) + lowByte;
      degrees = (heading / 10 + offset) % 360; 
      if (abs(degrees-initDegrees)>10){
        stopMovement();
        int rotationDegree = initDegrees-degrees;
        rotationDegree = (rotationDegree + 180) % 360 - 180;
        turn2(0, rotationDegree);
      }
      Serial.println(finDistance);
      printData();
    }
    if (wallDistance<=30) speed=100;
    else speed=200;
    analogWrite(Motor_L_pwm_pin, speed);
    analogWrite(Motor_R_pwm_pin, speed);
    statement1 = ((wallDistance != finDistance) && (wallDistance != finDistance+1) && (wallDistance != finDistance-1));
    statement2 = (wallDistance > 10 || (wallDistance<=10 && left==1));
  }
  stopMovement();
}

void lookaround(){
  turn2(0,-60);
}

void stopMovement(){//Stops the car
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}

void updatePulseCount() {//Pulse counter for right wheel
  pulseCount++;
}

