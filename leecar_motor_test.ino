#include <Wire.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
#include <SoftwareSerial.h>

const int motorInA = 7;
const int motorInB = 8;
const int motorPWM = 6;

const int steeringPin = 5;

const int handleMinAngle = 30;
const int handleMaxAngle = 150;
const int motorMaxSpeed = 255;
const int correctionFactor = 10;

int left = 125;
int right = 45;
int straight = 90;

Servo servo;

int speed = 100;  // 전역 변수로 선언

unsigned long previousMillis = 0;
const unsigned long interval = 10;

void setup() {
  pinMode(motorInA, OUTPUT);
  pinMode(motorInB, OUTPUT);
  analogWrite(motorPWM, 100);
  Serial.begin(9600);
  servo.attach(5);
}

void loop() {
  if (Serial.available()) {
    char in_data = Serial.read();
    Serial.print("data : ");
    Serial.println(in_data);

    switch (in_data) {
      case 'w':
        //increaseSpeed();
        SpeedUp();
         
        digitalWrite(motorInA, HIGH);
        digitalWrite(motorInB, LOW);
        break;
        
      case 'x':
        digitalWrite(motorInA, LOW);
        digitalWrite(motorInB, HIGH);
        break;
      case 's':
        digitalWrite(motorInA, HIGH);
        digitalWrite(motorInB, HIGH);
        servo.write(straight);
        break;
      case 'a':
        servo.write(left);
        delay(15);
        break;
      case 'd':
        servo.write(right);
        delay(15);
        break;
      default:
        break;
    }
  }
}


void SpeedUp(){
  speed += 10;
  analogWrite(motorPWM, speed);

  if (speed > motorMaxSpeed) {
      speed = motorMaxSpeed;
    }

  printf("speed: %d\n", speed);
  delay(10);
}
void increaseSpeed() {
  static int speed = 100;
  unsigned long currentMillis = millis();
 
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    speed += 1;
    if (speed > motorMaxSpeed) {
      speed = motorMaxSpeed;
    }

    analogWrite(motorPWM, speed);
    
  }
}
