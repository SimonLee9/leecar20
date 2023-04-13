#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>


//#include <Wire.h>
//#include <MPU6050_tockn.h>

ros::NodeHandle  nh;

const int motorInA = 7;
const int motorInB = 8;
const int motorPWM = 6;

const int SteeringPin = 5;

const int handleMinAngle = 30;
const int handleMaxAngle = 150;
const int motorMaxSpeed = 255;
const int correctionFactor = 10;

int left = 125;
int right = 45;
int straight = 90;

Servo Steering_servo;

int speed = 100;  // Àü¿ª º¯¼ö·Î ¼±¾ð

unsigned long previousMillis = 0;
const unsigned long interval = 10;

int Steering_vel = 0;
int Steering_angle = 90; // Init angle

void Steering_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  Steering_vel = cmd_msg.data[0]; //set servo angle, should be from 0-180 
  Steering_angle = cmd_msg.data[1];
  Steering_servo.write(Steering_angle);
  digitalWrite(13, HIGH-digitalRead(13));//toggle led


}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("leecar18_cmd_vel", Steering_cb);

void setup() {

  pinMode(motorInA, OUTPUT);
  pinMode(motorInB, OUTPUT);
  
  Serial.begin(9600);

  Steering_servo.attach(SteeringPin); // Pin number
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
    digitalWrite(motorInA,LOW);
    //digitalWrite(motorInA,LOW);

    analogWrite(motorPWM, 100);
    //digitalWrite(motorInA, motorA);
    //digitalWrite(motorInB, motorB);
    //servo.write(servoAngle);
    nh.spinOnce();
    delay(1);
}

