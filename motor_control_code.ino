/*
SparkFun Inventor’s Kit
Circuit 5B - Remote Control Robot

Control a two wheeled robot by sending direction commands through the serial monitor.
This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
Check out the rest of the book at
https://www.sparkfun.com/products/14326

This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
This code is completely free for any use.

View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;


//the right motor will be controlled by the motor A pins on the motor driver

const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

int switchPin = 7;             //switch to turn the robot on and off

const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 90 units, the robot turns about 90 degrees

                               //Note: these numbers will vary a little bit based on how you mount your motors, the friction of the
                               //surface that your driving on, and fluctuations in the power to the motors.
                               //You can change the driveTime and turnTime to make them more accurate

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String distance;               //the distance to travel in each direction

/********************************************************************************/
void setup()
{
  
   // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
{
  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);



 
 
}
}
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
/********************************************************************************/
void loop(){
  
  if(digitalRead(7) == LOW){
  IMU.readSensor();  // read the sensor
  float y =  IMU.getAccelY_mss(); 
  while (y!=-2.501){ 
  IMU.readSensor();  // read the sensor
  Serial.print("AccelY: ");  
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("  ");
  Serial.println();
  delay(100);
  float y =  IMU.getAccelY_mss();
  Serial.print(y);
  Serial.print("  ");
  Serial.println();   
/*The following section of code is designed to create a running average of values from the accelerometer. This was necessary to eliminate spikes in torque that created problems that caused t
 *the car to jerk back and forth in a way that did not allow for proper balance. 
*/
      float prev_y1 = y; 
      delay(10);
      float prev_y2 = y; 
      delay(10);      
      float prev_y3 = y;
      delay(10);
      float prev_y4 = y;
      delay(10);
      float prev_y5 = y;
      delay(10);      
      float prev_y6 = y;
      delay(10);
      float prev_y7 = y;
      delay(10);
      float prev_y8 = y;
      delay(10);      
      float prev_y9 = y;
      delay(10);
      float prev_y10 = y;
      delay(10);
      float prev_y11 = y;
      delay(10);      
      float prev_y12 = y;
      delay(10);
      float prev_y13 = y;
      delay(10);
      float prev_y14 = y;
      delay(10);      
      float prev_y15 = y;
      delay(10);
      float prev_y16 = y;
      delay(10);
      float prev_y17 = y;
      delay(10);      
      float prev_y18 = y;

      float avg_y = (prev_y1 + prev_y2 + prev_y3 + prev_y4 + prev_y5 + prev_y6 + prev_y7 + prev_y8 + prev_y9 + prev_y10 + prev_y11 + prev_y12 + prev_y13 + prev_y14 + prev_y15 + prev_y16 + prev_y17 + prev_y18) / 18;

      
      if (avg_y >= -2.8 && avg_y <= -2.2)                  // if statement to decide direction, seems to not recieve data from the accelerometer
    {
        rightMotor(0);                                    // drive the right wheel forward
        leftMotor(0); 
        Serial.println("motorstuff");                     // drive the left wheel forward
        // delay(driveTime * distance.toInt());           // drive the motors long enough travel the entered distance
        // rightMotor(0);                                 // turn the right motor off
        // leftMotor(0);
    }
      else if (avg_y > -2)
    {
        rightMotor(80);                                   // drive the right wheel forward
        leftMotor(80);                                    // drive the left wheel forward
       
        
    }
      else if (avg_y < -3)
    {
        rightMotor(-80);                                  // drive the right wheel backward
        leftMotor(-80);                                   // drive the left wheel backward
       
    }
      else
    {
      continue;
    }
  }
}
}
/********************************************************************************/
