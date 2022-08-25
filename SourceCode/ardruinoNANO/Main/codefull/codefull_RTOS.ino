
// ******************** Declare header*************************
#include <Arduino.h>
#include "ArdRTOS.h"
#include <ArduinoJson.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
//#include <semphr.h>
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>

//*************************************
#define TRIG 11   //trig pin of HC-SR04
#define ECHO 12   //echo pin of HC-SR04
#define IR_Sensor A0
#define START A7
#define IN1 4     //right
#define IN2 13    //right
#define IN3 8     //left
#define IN4 7
#define ENA 6
#define ENB 5
#define Encoder_R 2
#define Encoder_L 3
#define Rx 10             //PIN 9  _Rx
#define Tx 9             //PIN 10  _Tx  

//**************************************
MPU6050 mpu6050(Wire, 1.5, 0.7);
Servo myservo;
SoftwareSerial mySerial(Rx, Tx); //Khởi tạo cổng serial mềm
//SimpleKalmanFilter LocNhieu(0.55, 0.55, 0.01);


//************** VARIABLES **************
long x0;
int INzone = 0, x1 = 0, x2 = 0;
volatile unsigned int counterL;  //khai báo biến đếm encoder
volatile unsigned int counterR; //khai//int demMPU = 0;
unsigned long last;
float i_input = 0;
float d_last = 0;
float target_dir = 0.0;
float Kp = 1;
float Ki = 0.06;
float Kd = 0.08;
float z_axis;
int steer;

//****************************General setup*****************************
void setup() {
  //****************************General setup*****************************
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IR_Sensor, INPUT);  //infrared sensor
  pinMode(START, INPUT);
  myservo.attach(A3);  // attaches the servo on pin 9 to the servo object

  //*************************set up for MPU6050**************************
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //***************************setup sensor HC-SR04*****************************
  pinMode(TRIG, OUTPUT);  // trig pin is used to trigger the ultrasonic
  pinMode(ECHO, INPUT);   // Echo pin produces a pulse when the reflected signal is received

  //****************************** start serial port at 115200 bps:
  Serial.begin(115200);
  while (!Serial) {}
  mySerial.begin(115200);
  //***************************Create tasks of RTOS****************************
  OS.addTask(Obstacle, 256);
  OS.addTask(Motor_forward, 256);
  OS.addTask(Check_Zone, 256);
  OS.begin();
}

void loop() {}

void Motor_forward()
{
  for (;;) {
    SERVO(92, 10);
    if (x0 > 30 && analogRead(START) > 512) {
      target_dir = 0.2;
      Serial.print(" steer=");
      Serial.println(steer);
      steer = pid( target_dir - z_axis );
      MOTOR(109 + steer, 99 -  steer);
    }
    else if (INzone == 6 || x0 < 20)
      MOTOR(0, 0);
    OS.delay(100);
  }
}
void Check_Zone()
{
  for (;;) {
    mpu6050.update();
    z_axis = mpu6050.getAngleZ();
    Serial.print("z axis:    ");
    Serial.println(z_axis);
    x0 = Distance();
    Serial.println(x0);
    StaticJsonDocument<40> doc;
    int temp = 0;
    if (analogRead(IR_Sensor) > 512) {
      temp = 1;
    }
    x1 = x2;
    x2 = temp;
    if (x2 - x1 == 1)
      INzone++;
    doc["Zone"] = INzone;
    if (INzone > 6)
      INzone = 1;
    serializeJson(doc, mySerial);
    Serial.print("Zone: ");
    Serial.println(INzone);
    OS.delay(100);
  }
}

void Obstacle()
{
  for (;;) {
    if (x0 < 30 && INzone) {
      SERVO(180, 10);
      MOTOR(0, 0);
      if (x0 < 30 )
      {
        target_dir = 180;
        steer = pid( target_dir - z_axis );
        Serial.print(" steer=");
        Serial.println(steer);
        MOTOR(35 + steer, -steer);
      }
      else {
        target_dir = -180;
        steer = pid( target_dir - z_axis );
        Serial.print(" steer=");
        Serial.println(steer);
        MOTOR(-steer, 35 + steer);
      }
      OS.delay(100);
    }
  }
}
/*---------------------- FUNTIONs ---------------------*/
void MOTOR(int speed_L, int speed_R)
{
  if (speed_L >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed_L);
  }
  if (speed_L < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255 - speed_L);
  }
  if (speed_R >= 0) {
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
    analogWrite(ENB, speed_R);
  }
  if (speed_R < 0) {
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
    analogWrite(ENB, 255 - speed_R);
  }
}

void SERVO(int Angle, int dly)
{
  int sv = 92;
  if (Angle > sv) {
    for (sv; sv <= Angle; sv++) { // goes from Angle degrees to sv(90) degrees*/
      myservo.write(Angle);    // tell servo to go to position in variable 'sv'
      delay(dly); // waits dly (ms) for the servo to reach the position

    }
  }
  if (Angle < sv) {
    for (sv; sv >= Angle; sv--) { // goes from sv(90) degrees to Angle degrees
      myservo.write(sv);    // tell servo to go to position in variable 'sv'
      delay(dly);           // waits dly (ms) for the servo to reach the position
    }
  }
}

long Distance()
{
  // Clears the trigPin condition
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  unsigned long tg = pulseIn(ECHO, HIGH);
  long dis = tg  * 0.034 / 2;
  if (dis == 0)
    return 50;
  else
    return dis;
}
int pid(float error) {
  float p_input;
  float d_input;

  p_input = error;
  i_input = constrain(i_input + error, -50, +50);
  d_input = error - d_last; d_last = error;

  return p_input * Kp + i_input * Ki + d_input * Kd;
}
