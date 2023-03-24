# leecar20


이 코드는 아두이노를 사용하여 자동차를 조종하는 코드입니다. MPU6050 모듈을 사용하여 자동차의 기울기를 감지하고, 이를 이용해 모터 속도와 핸들 조향을 조절합니다. Servo 모터를 사용하여 핸들을 조향하며, 모터를 제어하는 데에는 L298N 모터 드라이버 모듈을 사용합니다.

코드에서는 setup() 함수에서 각 핀들의 입력/출력 방향을 설정하고, MPU6050 모듈과 Wire 라이브러리를 초기화합니다. 또한, servo 모터를 핀 3에 연결하고, 초기 각도를 90도로 설정합니다.

loop() 함수에서는 MPU6050 모듈을 이용해 현재 자동차의 기울기를 감지하고, 이를 이용해 모터 속도와 핸들 각도를 설정합니다. 또한, 시리얼 통신을 이용하여 외부에서 핸들 조향 값을 입력받을 수 있습니다. 모터 속도와 핸들 각도를 설정하는 함수는 각각 setMotorSpeed()와 setServoAngle()입니다. 모터 속도를 조정하는데에는 adjustMotorSpeed() 함수를 사용합니다.

printAngle() 함수는 현재 자동차의 각도를 시리얼 모니터에 출력하기 위해 사용됩니다.


### MPU-6050

https://github.com/tockn/MPU6050_tockn/blob/master/src/MPU6050_tockn.cpp
https://steemit.com/kr-arduino/@codingman/mpu-6050-processing


###VNH5019 Motor Driver

https://www.pololu.com/product/1451/specs
