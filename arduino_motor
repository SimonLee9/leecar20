#include <Wire.h>
#include <Servo.h>
#include <MPU6050_tockn.h>

const int motorPin1 = 9;
const int motorPin2 = 10;
const int speedPin = 6;
const int turnPin = 5;
const int steeringPin = 11;
const int handleMinAngle = 30;
const int handleMaxAngle = 150;
const int motorMaxSpeed = 150;
const int correctionFactor = 10;

MPU6050 mpu6050(Wire);
Servo servo;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(speedPin, INPUT);
  pinMode(turnPin, INPUT);

  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();

  servo.attach(3);
}

void loop() {
  mpu6050.update();

  int steeringValue = pulseIn(steeringPin, HIGH, 25000);
  int handleAngle = map(steeringValue, 0, 180, handleMinAngle, handleMaxAngle);

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
  if(motorSpeed > 1500) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(11, motorSpeed - 1500);
  } else if(motorSpeed < 1500) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(11, 1500 - motorSpeed);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(11, 0);
  }
}

void setServoAngle(int motorTurn) {
  if(motorTurn > 1500) {
    servo.write(map(motorTurn, 1500, 2000, 90, 180));
  } else if(motorTurn < 1500) {
    servo.write(map(motorTurn, 1000, 1500, 0, 90));
  } else {
    servo.write(90);
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
