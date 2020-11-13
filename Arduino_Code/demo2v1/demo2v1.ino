#include <math.h>
#include <Wire.h> 
#define ADDRESS 0x08
int nD2 = 4; // Motor Tristate
int motor1Dir = 7; //Better realized as Voltage Sign
int motor2Dir = 8; //Better reawlized as Voltage Sign
int motor1Speed = 9; //Better realized as Voltage of M1
int motor2Speed = 10; //Better realized as Voltage of M2
int motor1A = 2;
int motor1B = 6;
int motor2A = 3;
int motor2B = 4;
int nSF = 12; //Status Flag Indicator

//global state variable
int state = 1;

//xAngle variable
double xAngle;

//Distance variable
double distanceToBeacon;
// Time through to only capture 1 distance
int timeThrough;

long oldPosition  = -999;
const float pi = 3.1415926535898;


// Desired linear velocity for the motors
double desiredVelocity = 1;
double KpV = 3.34651664;
double KiV = 0.0038853330385174;;
double KdV = 2.5256354;
double deltaError = 0;
double totalVelocityError = 0;
double previousVelocityError = 0;
double newVelocityError = 0;
double controlVelocitySignal = 0;

// Desired linear position for the motors
double desiredPosition = 5 * .3048;
double KpD = 2.165965794;
double KiD = 4.652362528;
double totalError = 0;


double currentPosition = 0;

int data = 0;
int toSend = 0;


//variable to keep track of the counts for each wheel (encoder)
int countRight = 0;
int countLeft = 0;

//Counts per rotation
//The encoder has 1600 'clicks' or counts for one revolution
const int countsPerRot = 1600;

//Radius of Wheels (meters)
double radius = 0.073;

//Wheel Baseline (meters)
double d = 0.247;


//angular postion of the Right wheel
double rightWheelPosNew = 0;
double rightWheelPosOld = 0;

//angular postion of the Left wheel
double leftWheelPosNew = 0;
double leftWheelPosOld = 0;

//Time Variables for Right Wheel - these keep track of when the POSITIONS are read.
unsigned long rightTimeNew;
unsigned long rightTimeOld;
double rightDeltaT;

//Time Variables for Left Wheel  - these keep track of when the POSITIONS are read.
unsigned long leftTimeNew;
unsigned long leftTimeOld;
double leftDeltaT;

//Velocity of Right Wheel
double velocityRight;

//Velocity of Left Wheel
double velocityLeft;

//Angular velocity of the Right Wheel
double angVelocRight;

//Angular Velocity of the Left Wheel
double angVelocLeft; 

//New and Old Position of Robot, x, y, and phi coordinates, also a new delta t
double wheelXPos_new;

double wheelXPos_old;

double wheelYPos_new;

double wheelYPos_old;

double phi_new;

double phi_old;

double deltaT;

double timeOld;

//void sendData(){
//Wire.write(data);
//}




unsigned long currentTime = 0;
int samplePeriod = 10;

unsigned long oldTime;
void setup() {  

  //Right Wheel Enocoder Setup
  pinMode(motor1A, INPUT);
  pinMode(motor2B, INPUT);

  //Left Wheel Encoder Setup
  pinMode(motor2A, INPUT);
  pinMode(motor2B, INPUT);

  //Set Up Right Wheel Interrupt
  attachInterrupt(digitalPinToInterrupt(motor1A), rightWheelCount, CHANGE);
  //Set Up Left Wheel Interrupt
  attachInterrupt(digitalPinToInterrupt(motor2A), leftWheelCount, CHANGE);

  
  //Pins 4 Digital 4 - nD2 - triState
  pinMode(nD2, OUTPUT);
  
  //7,8 - Voltage Sign
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  //9, 10 Motor Voltage
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  
  //Pins 12 for input - nSF - Status flag indicator
  pinMode(nSF, INPUT);

  digitalWrite(nD2, HIGH);
  digitalWrite(motor1Dir, HIGH); //High for CW
  digitalWrite(motor2Dir, LOW); //Low for CCW

  //analogWrite(motor1Speed, HIGH);
  //analogWrite(motor2Speed, HIGH);

  //Initializes the serial monitor and sets up the Arduino for I2C communication
  Serial.begin(9600);
  Wire.begin(ADDRESS);
  Serial.println("Basic Encoder Test:");
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

}

void loop() {


//  double time = millis() / pow(10, 3);
//  double Vbar_a = 8;
//  // Gets the requested velocity from the outer loop controller
//  desiredVelocity = linearDistanceController(currentPosition);
//  // Scales the motor value to be compatible with the pwm system
//  int motorSpeed = double((linearController(-1*velocityRight) / 15.2)) * 255;
//  // Ensures the motor value doesn't leave the allowable limits
//  if(motorSpeed > 255){
//    motorSpeed = 255;
//  }else if(motorSpeed < 0){
//    motorSpeed = 0;
//  }
//
//  // Sends the actual commands to the motors
//  analogWrite(motor2Speed, motorSpeed);
//  analogWrite(motor1Speed, motorSpeed);
 // Wire.onReceive(receiveData);
 // Wire.onRequest(sendData);
  //acquire beacon state

  // This is the states where the robot acquires the beacon
  if (state == 1){
    xAngle = data;
    Serial.println(xAngle);
    // Rotate the robot in 45 degree increments to find the beacon in the camera view
    while(xAngle == 255){
      digitalWrite(motor1Dir, LOW); //High for CCW
      digitalWrite(motor2Dir, LOW); //Low for CCW
      int delayCount = (45 * 6.6071) + 57.5;
      analogWrite(motor2Speed, 110);
      analogWrite(motor1Speed, 110);
      delay(delayCount);
      analogWrite(motor2Speed, 0);
      analogWrite(motor1Speed, 0);
      delay(1000);
      xAngle = data;
      Serial.println(xAngle);
      if(xAngle != 255){
        xAngle = (xAngle/(double)4)-27;
      }
    }
    
    // Move the robot to face the beacon
    while((xAngle > 3) or (xAngle < -3)){
      
      // Convert the received data to an angle
      // Determine which way to spin based on the orientation of the robot with respect to the beacon
      if(xAngle > 0){
        digitalWrite(motor1Dir, LOW); //High for CCW
        digitalWrite(motor2Dir, LOW); //Low for CCW
      }if(xAngle < 0){
        digitalWrite(motor1Dir, HIGH); //LOW for CW
        digitalWrite(motor2Dir, HIGH); //HIGH for CW
      }
      Serial.println(xAngle);
      // Calculate the move the robot needs to make
      int delayCount = (abs(xAngle) * 6.6071) + 57.5;
      analogWrite(motor2Speed, 100);
      analogWrite(motor1Speed, 100);
      delay(delayCount);
      analogWrite(motor2Speed, 0);
      analogWrite(motor1Speed, 0);
      // Wait to get a high quality image and update the angle position variable
      delay(1000);
      xAngle = data;
      xAngle = (xAngle/(double)4)-27;      
    }
    if(((xAngle != 0) and (xAngle < 3) and (xAngle > -3))){
      // Once the robot is facing the beacon stop the motors and transition to the next program state
      analogWrite(motor1Speed, 0);
      analogWrite(motor2Speed, 0);
      state = 2;
      delay(700);
    }
    
  }
  // This is the state where the robot moves to the beacon
  else if(state == 2){
    if(timeThrough == 1){
      distanceToBeacon = data;
      distanceToBeacon = distanceToBeacon * (304.8/254);
      distanceToBeacon = distanceToBeacon - 10;
      distanceToBeacon = distanceToBeacon * 0.0328084;
      timeThrough = 0;
    }
    
    digitalWrite(motor1Dir, HIGH); //High for CW
    digitalWrite(motor2Dir, LOW); //Low for CCW
    double time = millis() / pow(10, 3);
    double Vbar_a = 8;
    // Gets the requested velocity from the outer loop controller
    desiredVelocity = linearDistanceController(currentPosition);
    // Scales the motor value to be compatible with the pwm system
    int motorSpeed = double((linearController(-1*velocityRight) / 15.2)) * 255;
    // Ensures the motor value doesn't leave the allowable limits
    if(motorSpeed > 255){
      motorSpeed = 255;
    }else if(motorSpeed < 0){
       motorSpeed = 0;
    }

    // Sends the actual commands to the motors
    analogWrite(motor2Speed, motorSpeed);
    analogWrite(motor1Speed, motorSpeed);
  


    double rhoDot = -1 * velocityRight;
    if((distanceToBeacon <= 20) and (distanceToBeacon != 0)){
      analogWrite(motor1Speed, 0);
      analogWrite(motor2Speed, 0);
      state = 3;
    }
  }
  // This is the state where the robot circles the beacon
  else if(state == 3){
    
  }

 
 
}

//ISR for the right wheel/encoder
void rightWheelCount(){

rightTimeNew = micros();

//Right wheel is moving CCW
if (digitalRead(motor1B) == digitalRead(motor1A)){
  countRight -= 1;
}

//Right wheel is moving CW
if (digitalRead(motor1B) != digitalRead(motor1A)){
  countRight += 1;
}

//Calcuate Time spent inside the ISR, and then calculate angular velocity and velocity

rightDeltaT = (double)rightTimeNew - (double)rightTimeOld;
}
////////////////////////////////////////////////////////////

//ISR for the left wheel/encoder
void leftWheelCount(){
leftTimeNew = micros();
//Left wheel is moving CCW
if (digitalRead(motor2B) == digitalRead(motor2A)){
  countLeft -= 1;
}
//Left wheel is moving CW
if (digitalRead(motor2B) != digitalRead(motor2A)){
  countLeft += 1;
}
leftDeltaT = ((double)leftTimeNew - (double)leftTimeOld);
}
///////////////////////////////////////////////////////////////////////////////////////////
double linearController(double velocityRight){
  // New error is used in the proportional term
  float newVelocityError = desiredVelocity - velocityRight;
  // For the integral part
  totalVelocityError += newVelocityError;
  // For the derivative part
  deltaError = newVelocityError - previousVelocityError;
 
  // Derivative portion of the controller
  double derivativePart;
  if(rightDeltaT < .001){
    derivativePart = 0;
  }else{
    derivativePart = (KdV*deltaError) / ((rightDeltaT+10)/1000000);
  }
  // Used for debugging purposes
  //Serial.println(derivativePart);
  // Calculates the value to send from the controller
  double controlVelocitySignal = (KpV*newVelocityError) + (KiV*(rightDeltaT/1000000)*totalVelocityError) + derivativePart;
  return controlVelocitySignal; 
}

double linearDistanceController(double currentPosition){
  // New error is used in the proportional term
  float newDistanceError = (desiredPosition - currentPosition);
  // Total error is used in the integration term 
  totalError += newDistanceError;
  // Calculates the new value based on the current and desired position
  double controlSignal = (KpD*newDistanceError) + (KiD*(((double)rightTimeNew - (double)rightTimeOld)/1000000)*totalError);
  return controlSignal;
}

void receiveData(int byteCount){
  // Reads in data over the I2C communication line when data is available
  while(Wire.available()){
    data = Wire.read();
    
  }
  // Resets the total error term for the PI controller
  totalError = 0;
}

// This is the function that sends data over the I2C communication line
void sendData(){
  // Sends the current position in radians over the communication line to be displayed on the LCD screen
  Wire.write(state);
}
void rotateRight(){
    digitalWrite(motor1Dir, LOW); //Low for CCW
    digitalWrite(motor2Dir, LOW);
    analogWrite(motor1Speed, 25);
    analogWrite(motor2Speed, 25);
}
void rotateLeft(){
    digitalWrite(motor1Dir, HIGH); //High for CW
    digitalWrite(motor2Dir, HIGH);
    analogWrite(motor1Speed, 25);
    analogWrite(motor2Speed, 25);
}
