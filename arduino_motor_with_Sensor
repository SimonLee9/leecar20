#include <Wire.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
#include <SoftwareSerial.h>

const int motorInA = 2;
const int motorInB = 3;
const int motorPWM = 9;
const int speedPin = 6;
const int turnPin = 5;

const int steeringPin = 11;
const int handleMinAngle = 30;
const int handleMaxAngle = 150;
const int motorMaxSpeed = 255;
const int correctionFactor = 10;

MPU6050 mpu6050(Wire);
Servo servo;
SoftwareSerial bluetoothSerial(2, 3); // RX, TX

void setup() {
  pinMode(motorInA, OUTPUT);
  pinMode(motorInB, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(speedPin, INPUT);
  pinMode(turnPin, INPUT);

  Serial.begin(9600);
  bluetoothSerial.begin(9600);


  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();

  servo.attach(3);
}

void loop() {
  mpu6050.update();

  int steeringValue = pulseIn(steeringPin, HIGH, 25000);
  int handleAngle = map(steeringValue, 0, 180, handleMinAngle, handleMaxAngle);

  while (bluetoothSerial.available()) {
    char command = (char)bluetoothSerial.read();
    if (command == 'F') {
      setMotorSpeed(motorMaxSpeed);
    } else if (command == 'B') {
      setMotorSpeed(-motorMaxSpeed);
    } else if (command == 'L') {
      setServoAngle(handleMinAngle);
    } else if (command == 'R') {
      setServoAngle(handleMaxAngle);
    } else if (command == 'C') {
      setServoAngle((handleMinAngle + handleMaxAngle) / 2);
    }
  }

  if (Serial.available() > 0) {
    steeringValue = Serial.parseInt();
  }

  int motorSpeed = pulseIn(speedPin, HIGH, 25000);
  int motorTurn = pulseIn(turnPin, HIGH, 25000);

  setMotorSpeed(motorSpeed);
  setServoAngle(motorTurn);
  adjustMotorSpeed(mpu6050.getAngleX());

  delay(20);
  printAngle(mpu6050.getAngleX(), mpu6050.getAngleY(), mpu6050.getAngleZ());
}

void setMotorSpeed(int motorSpeed) {
  if (motorSpeed > 1500) {
    digitalWrite(motorInA, HIGH);
    digitalWrite(motorInB, LOW);
    analogWrite(motorPWM, map(motorSpeed, 1500, 2000, 0, motorMaxSpeed));
  } else if (motorSpeed < 1500) {
    digitalWrite(motorInA, LOW);
    digitalWrite(motorInB, HIGH);
    analogWrite(motorPWM, map(motorSpeed, 1000, 1500, 0, motorMaxSpeed));
  } else {
    digitalWrite(motorInA, LOW);
    digitalWrite(motorInB, LOW);
    analogWrite(motorPWM, 0);
  }
}

void setServoAngle(int motorTurn) {
  if(motorTurn > 1500) {
    servo.write(map(motorTurn, 1500, 2000, 90, 180));
  } else if(motorTurn < 1500) {
    servo.write(map(motorTurn, 1000, 1500, 0, 90));
  } else {
    //servo.write(90);
    
    // 이 부분이 추가된 부분입니다.
    if (servo.read() > 90) {
      servo.write(servo.read() - 1);
    } else if (servo.read() < 90) {
      servo.write(servo.read() + 1);
    }
  }
}

void adjustMotorSpeed(double gyroX) {
  int correction = -gyroX * correctionFactor;
  analogWrite(11, max(0, min(255, analogRead(11) + correction)));
}

void printAngle(float x, float y, float z) {
  Serial.print("angleX : ");
  Serial.print(x);
  Serial.print("\tangleY : ");
  Serial.print(y);
  Serial.print("\tangleZ : ");
  Serial.println(z);
}
