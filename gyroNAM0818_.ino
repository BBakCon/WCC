/* 수신 */
/*MEGA 보드 이용*/
/*서보모터 떨림 방지*/

#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Wire.h>

RF24 radio(26, 27); // SPI 버스에 nRF24L01 라디오를 설정하기 위해 CE, CSN 선언.
const byte address[6] = "77777"; //주소값을 5가지 문자열로 변경할 수 있으며, 송신기과 수신기가 동일한 주소로 해야됨.

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

//MPU6050 mpu;
//const int MPU = 0x68; // I2C address of the MPU-9155
int16_t ax, ay, az;
int16_t gx, gy, gz;

//8.18 변수명 선언
#define MPU 0x68
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;

double angleAcX2, angleAcY2, angleAcZ2;
double angleGyX2, angleGyY2, angleGyZ2;
double angleFiX2, angleFiY2, angleFiZ2;

const double RADIAN_TO_DEGREE = 180 / 3.14159;
const double DEGREE_PER_SECOND = 32767 / 250;
const double ALPHA = 1 / (1 + 0.0039);
const double ALPHA2 = 1 / (1 + 0.4);

unsigned long now = 0;
unsigned long past = 0;
double dt = 0;

unsigned long now2 = 0;
unsigned long past2 = 0;
double dt2 = 0;

double baseAcX, baseAcY, baseAcZ;
double baseGyX, baseGyY, baseGyZ;

double baseAcX2, baseAcY2, baseAcZ2;
double baseGyX2, baseGyY2, baseGyZ2;

Servo servoPF; // 앞 쪽 피치 각도를 위한 서보모터
Servo servoPB; // 뒷 쪽 피치 각도를 위한 서보모터
Servo servoRF; // 앞 쪽 롤 각도를 위한 서보 모터
Servo servoRB; // 뒷 쪽 롤 각도를 위한 서보 모터
Servo servoSL; // 왼쪽 조향 각도
Servo servoSR; // 오른쪽 조향 각도

Servo servoT1; // 스탬프 1
Servo servoT2; // 스탬프 2

Servo esc_b; // 프로펠러 모터
Servo esc_r; // 프로펠러 모터

int black = 0;
int red = 0;

int slope_A; // slope_A 는 벽에 붙어 있을때의 차체 기울어진 각도
int slope_a;
float th_R; // roll 부분의 회전 각도 radian
float th_P; // PItch 부분 회전 각도 radian
float TH_R; // roll 부분의 회전 각도 60분법
float TH_P; // PItch 부분의 회전 각도 60분법
float mA =50; // 차체가 붙어있기 위한 최적 각도 *무게와 마찰계수에 따라 이 값만 바꿔주면 됨*

struct Data {
  byte joyX; // [joystick 으로 부터 받은 값, dc 모터 조종, 앞 뒤로 움직이기 위한 receive 값]
  byte joyY; // [joystick 으로 부터 받은 값, 역할 미정, ]
  byte poten1; // [가변 저항으로 부터 받은 값, 프로펠러 추력 결정, 프로펠러 추력 올리고 내리기 가능]
  byte poten2; // [가변 저항으로 부터 받은 값, 바퀴 조향 결정, 바퀴 좌우 움직임 가능]
  byte btnn1; // [버튼 으로 부터 받은 값, 특수 상황(바닥->벽 가는 상황, 벽->바닥 가는 상황), 오르고 내리기 가능]
  byte btnn2;
};
Data data;

int val_joyX;
int val_joyY;
int val_poten1;
int val_poten2;
int val_btnn1;
int val_btnn2;
int pos1;
int pos2;
int count1;
int count2;
int count3;

float CRVTP = 0; //currentvar_TP
float CRVTR = 0; //currentvar_TR
float PRVTP = 0; //previousvar_TP
float PRVTR = 0; //previousvar_TR

int PFS_In = -15;
int PBS_In = 90;

//float sensitivity = 0.1;

void prop();
void gyro();

//PID constants
//double kp = 0.075;
//double ki = 0.00001;
//double kd = 0.01;
//
//unsigned long currentTime, previousTime;
//double elapsedTime;
//double error;
//double lastError;
//double input, output, setPoint;
//double cumError, rateError;

void setup() {

  initMPU();
  Serial.begin(9600);
  calibrateSensor();
  past = millis();

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN); //전원공급에 관한 파워레벨을 설정합니다. 모듈 사이가 가까우면 최소로 설정합니다.
  //거리가 가까운 순으로 RF24_PA_MIN / RF24_PA_LOW / RF24_PA_HIGH / RF24_PA_MAX 등으로 설정할 수 있습니다.
  //높은 레벨(거리가 먼 경우)은 작동하는 동안 안정적인 전압을 가지도록 GND와 3.3V에 바이패스 커패시터 사용을 권장함
  radio.startListening(); //모듈을 수신기로 설정

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-9155)
  Wire.endTransmission(true);

  pinMode(22, OUTPUT);       // Motor A 방향설정1
  pinMode(23, OUTPUT);       // Motor A 방향설정2
  pinMode(24, OUTPUT);       // Motor B 방향설정1
  pinMode(25, OUTPUT);       // Motor B 방향설정2

  esc_b.attach(9, 1000, 2000); // 프로펠러 모터
  esc_r.attach(8, 1000, 2000); // 프로펠러 모터
  Serial.setTimeout(10);
  esc_b.write(0);
  esc_r.write(0);

  servoPF.attach(38); //앞 쪽 피치 각도를 위한 서보모터
  servoPB.attach(39); //뒷 쪽 피치 각도를 위한 서보모터
  servoRF.attach(40); //앞 쪽 롤 각도를 위한 서보모터,
  servoRB.attach(41); //뒷 쪽 롤 각도를 위한 서보모터
  servoSL.attach(42); //오른 쪽 조향 각도를 위한 서보모터 좌우 바꾸는게 더 낫더라구..?
  servoSR.attach(43); //왼 쪽 조향 각도를 위한 서보모터

  servoT1.attach(44); //
  servoT2.attach(46); //

  servoT1.write(90);
}

void loop() {

  //    servoPF.write(PFS_In+0); //Front roll servo ★ 초기 앞 피치 서보모터 각도 설정  (PF,PB,0,0)     10시, 6시  //+뒤의 숫자의 범위 0<x<90
  //    servoPB.write(PBS_In+0); //Back roll servo ★ 초기 뒤 피치 서보모터 각도 설정  (PF,PR,180,180) 3시, 11시   //+뒤의 숫자의 범위 0<x<90
  //    servoRF.write(90+50); //Front roll servo ★
  //    servoRB.write(87+50); //Back roll servo ★

  radiorec(); // radio 통신을 이용하여 조종기로 부터 값을 받아냄

  getADC(); // mpu6050 으로부터 가속도 및 자이로 값을 받아냄
  gyroSTR(); // gyroSTR로 slope_a 계산후 gyro()에 대입

  gyro(); // slope_a 값을 받아서 TH_P 와 TH_R 에 넣어주는 코드

  prop(); // 프로펠러 작동 시키는 함수
  wheelC(); // 바퀴 작동
  servoC(); // 서보모터 Servo__.write(__);

  btnnWork(); // 버튼 일에 대한 함수

}

// [특수 상황 1]
// 버튼을 일정 시간 이상 눌렀을 때 차체가 올라가기 위해서 프로펠러의 각도가 자동으로 돌아간다.
void climbwallF() {
  count2++;
  delay(10);
  Serial.println(count2);
  if (count2 > 50) {
    radiorec();
    for (pos1 = 180; pos1 > 150; pos1--) {
      if (pos1 == 180) {
        for (int i = 0; i < 30; i++) {
          //          servoPF.attach(38); //앞 쪽 피치 각도를 위한 서보모터
          //          servoPB.attach(39); //뒷 쪽 피치 각도를 위한 서보모터
          //          servoRF.attach(40); //앞 쪽 롤 각도를 위한 서보모터
          //          servoRB. attach(41); //뒷 쪽 롤 각도를 위한 서보모터
          servoPF.write(PFS_In + pos1 - 0); //Front pitch servo
          servoPB.write(PBS_In - 90); //Back pitch servo
          //          servoRF.write(90+0); //Front roll servo ★
          //          servoRB.write(87+0); //Back roll servo ★
          delay(10);
          wheelC();
          prop();
          radiorec();
        }
      }
      radiorec();
      prop();
      wheelC();
      servoPF.write(0 + pos1); //Front pitch servo
      servoPB.write(180 - pos1); //Back pitch servo
      //      servoRF.write(90+0); //roll servo ★
      //      servoRB.write(87+0); //Back roll servo ★
      Serial.println(pos1);
      delay(50);
    }
  }
  servoC();
}

void climbwallS() {
  count2++;
  delay(10);
  Serial.println(count2);
  if (count2 > 50) {
    radiorec();
    for (pos2 = 120; pos2 > 90; pos2--) {
      if (pos2 == 120) {
        for (int j = 0; j < 110; j++) {
          servoPF.attach(38); //앞 쪽 피치 각도를 위한 서보모터
          servoPB.attach(39); //뒷 쪽 피치 각도를 위한 서보모터
          //          servoRF.attach(40); //앞 쪽 롤 각도를 위한 서보모터
          //          servoRB.attach(41); //뒷 쪽 롤 각도를 위한 서보모터
          servoPF.write(PFS_In + pos2 - 30); //Front pitch servo
          servoPB.write(PBS_In + 90); //Back pitch servo
          //          servoRF.write(90+0); //Front roll servo ★
          //          servoRB.write(87+0); //Back roll servo ★
          delay(10);
          wheelC();
          prop();
          radiorec();
        }
      }
      radiorec();
      prop();
      wheelC();
      servoPF.write(PFS_In + pos2 - 40); //Front pitch servo
      servoPB.write(pos2); //Back pitch servo
      //      servoRF.write(90+0); //roll servo ★
      //      servoRB.write(87+0); //Back roll servo ★
      Serial.println(pos2);
      delay(50);
    }
  }
  servoC();
}

// [특수 상황 2]
// 버튼을 일정 시간 이상 눌렀을 때 차체가 내려가기 위해서 프로펠러의 각도가 자동으로 돌아간다.
void climbdown() {

}


//gyrobeg함수 무효화
/*
  // 자이로 값을 받는 함수
  void gyrobeg() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  //가속도 센서 (Accelator)
  //중력 가속도를 이용하여 기울어져 있는 정도를 나타낸다. X,Y,Z 축 방향과 일치한다.
  ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //  Serial.print("ax = "); Serial.print(ax);
  //  Serial.print(" | ay = "); Serial.print(ay);
  //  Serial.print(" | az = "); Serial.print(az);
  delay(1);

  if (ay >= 0) {
    slope_a = map(az, 17000, -17000, 0, 180); // 차체가 반시계로 회전하는 것을 + 부호로 수직 상태를 0이라 설정
  }                                                // 값을 0~180도 까지 설정
  if (ay < 0) {
    slope_a = map(az, 17000, -17000, 0, -179); // 차체가 시계방향으로 회전하는 것을 - 부호로 0~-179도 까지
  }
  //  Serial.print("ax = "); Serial.print(ax);
  //  Serial.print(" | ay = "); Serial.print(ay);
  //  Serial.print(" | az = "); Serial.println(az);

  Serial.print(slope_a); // 확인을 위한 시리얼 값 확인
  Serial.print("    ");

  }*/

//PID계산 무효화
/*
  double computePID(double inp) {  // PID 계산은 반드시 내부 루프 함수에 있어야 하며,
  // 함수 시작시에는 수행시간(elapsed time)을 설정해주어야 함
  // 현재 시간은 millis()로 얻음
  currentTime = millis(); //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

    Serial.print("time : ");
    //Serial.print(elapsedTime); 노란색선 때문에 주석처리함 , 8/17
    Serial.println("    ");

  error = setPoint - inp; // determine error
  cumError += error * elapsedTime; // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = slope_A - ( kp * error + ki * cumError + kd * rateError) ; //PID output

  lastError = error; //remember current error
  previousTime = currentTime; //remember current time

  return out; //have function return the PID output
  }
*/

//기존 자이로 함수 무효화

void gyro() {

  //  input = slope_a; //read from rotary encoder connected to A0
  //
  //  setPoint = slope_A; //set point at zero degrees
  //
  //  slope_A = computePID(input);
  //  Serial.print("||   after:  ");
  //  Serial.println(slope_A);

  th_R = asin(-sin(mA * PI / 180) * sin(slope_a * PI / 180));
  th_P = acos(cos(mA * PI / 180) / cos(th_R));

  TH_R = th_R * 180 / PI;
  TH_P = th_P * 180 / PI;
  if (ay >= 0 && slope_a > 90) {
    TH_P = -TH_P;
  }
  if (ay < 0 && slope_a < -90) {
    TH_P = -TH_P;
  }

  //    servoPF.write(90-TH_P); //pitch servo
  //    servoPB.write(90-TH_P); //pitch servo
  //    servoR.write(90-TH_R); //roll servo

  //  Serial.print("   ");
  //  Serial.print(TH_P);
  //  Serial.print("   ");
  //  Serial.print(TH_R);
  //  Serial.println("   ");

}


//피치 및 롤 부분 서보모터를 돌리기 위한 함수
void servoC() {

  //    servoRF.write(90+50); //Front roll servo ★
  //    servoRB.write(87+50); //Back roll servo ★
  CRVTP = constrain(TH_P, -90, 90);
  //  Serial.print(CRVTP);
  //  Serial.print("  ");
  CRVTR = constrain(TH_R, -90, 90);
  //  Serial.println(CRVTR);

  if (angleFiX2 > -55) {     // 로봇이 벽에 붙어 있을 때
    //    if ((PRVTP + 1 < CRVTP and CRVTP < PRVTP + 60) or (PRVfTP - 60 < CRVTP and CRVTP < PRVTP - 1)) {
    //      if((PRVTP+2 < CRVTP) or (CRVTP < PRVTP-2)){
    if (CRVTP > 0) {
      servoPF.write(CRVTP - 15); //pitch servo   (PF,PB,0,0)     10시, 6시  [서보모터의 위치를 화살표의 머리라 생각하겠음]
      servoPB.write(CRVTP + 90); //pitch servo   (PF,PR,180,180) 3시, 11시
      //        PRVTP = CRVTP;
    }
    if (CRVTP < 0) {
      servoPF.write(0);
      servoPB.write(CRVTP + 90); //pitch servo   (PF,PR,180,180) 3시, 11시
    }

    //    }
    //    if ((PRVTR + 1 < CRVTR and CRVTR < PRVTR + 70) or (PRVTR - 70 < CRVTR and CRVTR < PRVTR - 1)) {

    servoRF.write(90 - CRVTR); //roll servo ★
    servoRB.write(87 - CRVTR); //roll servo ★
    delay(10);
    //      PRVTR = CRVTR;
    //    }

  }

  if (angleFiX2 <= -55) {  // 로봇이 바닥에 있을 때
    //    servoPF.attach(38); //앞 쪽 피치 각도를 위한 서보모터
    //    servoPB.attach(39); //뒷 쪽 피치 각도를 위한 서보모터
    //    servoRF.attach(40); //앞 쪽 롤 각도를 위한 서보모터
    //    servoRB.attach(41); //뒷 쪽 롤 각도를 위한 서보모터
    servoPF.write(-15 + 45); //Front pitch servo
    servoPB.write(90 + 45); //Back pitch servo
    servoRF.write(90 + 0); //Front roll servo ★
    servoRB.write(87 + 0); //Back roll servo ★
  }

  //    Serial.print("    ");
  //    Serial.print(CRVTP);
  //    Serial.print("    ");
  //    Serial.println(CRVTR);
}

// RF 통신을 위한 함수
void radiorec() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data));
  }
  val_joyX = data.joyX;
  val_joyY = data.joyY;
  val_poten1 = data.poten1;
  val_poten2 = data.poten2;
  val_btnn1 = data.btnn1;
  val_btnn2 = data.btnn2;

  //      Serial.print(val_joyX); Serial.print("  ");
  //      Serial.print(val_joyY); Serial.print("  ");
  //      Serial.print(val_poten1); Serial.print("  ");
  //      Serial.print(val_poten2); Serial.print("  ");
  //      Serial.print(val_btnn1);  Serial.print("  ");
  //      Serial.println(val_btnn2);
}

// 프로펠러를 조종하기 위한 함수
void prop() {
  black = val_poten1;
  red = val_poten1;
  esc_b.write(black);
  esc_r.write(red);
}

void wheelC() {
  if (val_joyX > 100)     forw();
  if (val_poten2 < 88)    steering_L();
  if (val_poten2 > 92)    steering_R();
  if (val_joyX < 80)      back();
  //  else if (rx_data == B_BACKWARD_L)  back_l();
  //  else if (rx_data == B_BACKWARD_R)  back_r();
  //  else if (rx_data == B_CLOCKWISE)   clockwise();
  //  else if (rx_data == B_CCLOCKWISE)  cclockwise();
  if (val_joyX > 80 and val_joyX < 100)    sstop();
}

//바퀴를 앞으로 돌린다
void forw() {
  /*모터A설정*/
  digitalWrite(22, HIGH);     // Motor A 방향설정1
  digitalWrite(23, LOW);      // Motor A 방향설정2
  analogWrite(4, 255);       // Motor A 속도조절 (0~255)
  /*모터B설정*/
  digitalWrite(24, LOW);      // Motor B 방향설정1
  digitalWrite(25, HIGH);     // Motor B 방향설정2
  analogWrite(5, 255);        // Motor B 속도조절 (0~255)
}

//조향장치를 왼쪽으로 돌린다
void steering_L() {
  servoSL.write(90 - (val_poten2 - 90) * 1); servoSR.write(90 - (val_poten2 - 90) * 2 / 3);
}

//조향장치를 오른쪽으로 돌린다
void steering_R() {
  servoSL.write(90 - (val_poten2 - 90) * 2 / 3); servoSR.write(90 - (val_poten2 - 90) * 1);
}

//바퀴를 뒤로 돌린다
void back() {
  /*모터A설정*/
  digitalWrite(22, LOW);      // Motor A 방향설정1
  digitalWrite(23, HIGH);     // Motor A 방향설정2
  analogWrite(4, 255);      // Motor A 속도조절 (0~255)
  /*모터B설정*/
  digitalWrite(24, HIGH);    // Motor B 방향설정1
  digitalWrite(25, LOW);     // Motor B 방향설정2
  analogWrite(5, 255);      // Motor B 속도조절 (0~255)
}

void sstop() {
  /*모터A설정*/
  digitalWrite(22, LOW);     // Motor A 방향설정1
  digitalWrite(23, LOW);      // Motor A 방향설정2
  analogWrite(4, 0);       // Motor A 속도조절 (0~255)
  /*모터B설정*/
  digitalWrite(24, LOW);      // Motor B 방향설정1
  digitalWrite(25, LOW);     // Motor B 방향설정2
  analogWrite(5, 0);        // Motor B 속도조절 (0~255)
}

void stamp() {

  for (int p = 1; p < 60; p ++) {
    servoT1.write(90 + p); //Front pitch servo
    servoT2.write(180 - p); //Back pitch servo
    delay(15);
    //    gyrobeg();
    //    gyro();
    servoC();
    radiorec();
    prop();
    wheelC();
  }
  count2 = 0;

}

//8.18. 자이로 함수 첨가
void gyroSTR() {
  //  getADC();
  getDT();
  // Yaw
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
  angleAcY *= RADIAN_TO_DEGREE;

  double angleTmp = (GyY - baseGyY) * dt;
  angleTmp /= DEGREE_PER_SECOND;
  angleGyY += angleTmp;

  angleTmp = angleFiY + angleTmp;
  angleFiY = ALPHA * angleTmp + (1.0 - ALPHA) * angleAcY;

  // Pitch
  angleAcX2 = atan(-AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
  angleAcX2 *= RADIAN_TO_DEGREE;

  getDT2();
  double angleTmp2 = (GyX - baseGyX) * dt2;
  angleTmp2 /= DEGREE_PER_SECOND;
  angleGyX2 += angleTmp2;

  angleTmp2 = angleFiX2 + angleTmp2;
  angleFiX2 = ALPHA2 * angleTmp2 + (1.0 - ALPHA2) * angleAcX2;


  Serial.print("Pitch slope: ");
  Serial.print(angleFiX2);
  Serial.print("  ");

//  Serial.print("Yaw slope: ");
//  Serial.print(angleFiY);
//  Serial.print("  ");

  slope_a = 1.08 * (angleFiY - 3);
  Serial.print("slope_a : ");
  Serial.println(slope_a);

}

void getDT() {
  now = millis();
  dt = (now - past) / 1000.0;
  past = now;
}

void getDT2() {
  now2 = millis();
  dt2 = (now2 - past2) / 1000.0;
  past2 = now2;
}

void calibrateSensor() {
  double sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  getADC();
  for (int i = 0; i < 10 ; i++) {
    getADC();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }
  baseAcX = sumAcX / 10;
  baseAcY = sumAcY / 10;
  baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10;
  baseGyY = sumGyY / 10;
  baseGyZ = sumGyZ / 10;
}

void initMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void getADC() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}
void btnnWork() {

  servoT1.write(90);
  servoT2.write(180);

  if (angleFiX2 <= -55) {
    if (val_btnn2 == 210) {
      count2++;
      delay(10);
      Serial.print("L");
      Serial.println(count2);
      if (count2 > 40) {
        climbwallF();
      }
    }
  }
  if (val_btnn2 == 200) {
    count2 = 0;
  }

  if (val_btnn1 == 250) {
    count1++;
    delay(10);
    Serial.print("R");
    Serial.println(count1);
    if (count1 > 20) {
      stamp();
    }
  }
  if (val_btnn1 == 240) {
    count1 = 0;
  }
}


//
//void back_l()
//{
//  digitalWrite(motor_a_dir, BACKWARD); analogWrite(motor_aZ_pwm, SPEED_50);
//  digitalWrite(motor_b_dir,DD BACKWARD); analogWrite(motor_b_pwm, SPEED_80);
//}
//
//void back_r()
//{
//  digitalWrite(motor_a_dir, BACKWARD); analogWrite(motor_a_pwm, SPEED_80);
//  digitalWrite(motor_b_dir, BACKWARD); analogWrite(motor_b_pwm, SPEED_50);
//}
//
//void clockwise()
//{
//  digitalWrite(motor_a_dir, BACKWARD); analogWrite(motor_a_pwm, SPEED_80);
//  digitalWrite(motor_b_dir, FORWARD); analogWrite(motor_b_pwm, SPEED_80);
//}
//
//void cclockwise()
//{
//  digitalWrite(motor_a_dir, FORWARD); analogWrite(motor_a_pwm, SPEED_80);
//  digitalWrite(motor_b_dir, BACKWARD); analogWrite(motor_b_pwm, SPEED_80);
//}
