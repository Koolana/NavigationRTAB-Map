//Использование выводов
/*
===PWM==============================================================
Моторы
4 ---> правый мотор PWM1 (+ земля)
5 ---> левый мотор PWM2
===DIGITAL==========================================================
Моторы
52 ---> Direction-пин правого мотора DIR1
53 ---> Direction-пин левого мотора DIR2
===ANALOG==========================================================
нет
*/
#include "motor.h"
#include "odometer.h"

//Encoder variables
const byte encoderRpinA = 2;                              //A pin -> the interrupt pin (2)
const byte encoderRpinB = 17;                              //B pin -> the digital pin (16)
const byte encoderLpinA = 3;                              //A pin -> the interrupt pin (3)
const byte encoderLpinB = 16;                              //B pin -> the digital pin (17)

// Timer variables
const double rate = 10;  // Hz
const double dT = (1 / rate);  // 100 ms = 10 times per sec - Timer interrupt interval
const long Timer1Interval = long(dT * 1000000);  // период счета

//Motor control variables
const int MotorRdir = 52;  // 4  //Right motor Direction Control pin
const int MotorLdir = 53;  // 7  //Left motor Direction Control pin
const int MotorRpwm = 4;  // 5   //Right motor PWM Speed Control pin
const int MotorLpwm = 5;  // 6   //Left motor PWM Speed Control pin

double R = 0.068;
double L = 0.273;

double maxSpeed = 0.544;  // максимальная линейная скорость при скважности 100%, в м/с
unsigned int PPR = 1450;  // импульсов энкодера за оборот

double CL = 1;  // 0.939809;  // корректирующие коеффициенты для радиусов колес
double CR = 1;  // 1.06019;

unsigned int prevTimeMicros = 0;

void printValue(double val, const char* namVal = NULL, bool withSignAndDouble = true);

Motor::MotorPins motorPinsR = {MotorRdir, MotorRpwm};
Motor::MotorPins motorPinsL = {MotorLdir, MotorLpwm};
Motor::PIDcoefs pid = {1, 5, 0};
Motor::MotorParams params = {maxSpeed, R, PPR};

ros::NodeHandle nh;

Motor motorR(motorPinsR, pid, params);
Motor motorL(motorPinsL, pid, params);

Odometer odometer(L, dT);

void cmd_velCallback(const geometry_msgs::Twist& toggle_msg);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCallback);

void setup() {
  nh.initNode();

  odometer.setupPublisher(nh);
  nh.subscribe(sub_cmd_vel);

  EncoderInit();
}

void loop() {
  motorR.update();
  motorL.update();

  if (abs(micros() - prevTimeMicros) > (1/rate) * 1000000) {
    odometer.update(motorL.getVelocity(), motorR.getVelocity());
    odometer.publish(nh.now());

    prevTimeMicros = micros();
  }

  nh.spinOnce();
  delay(1);
}

 void cmd_velCallback(const geometry_msgs::Twist& twist_msg) {
  double LinearVelocity   = twist_msg.linear.x;
  double AngularVelocity  = twist_msg.angular.z;

  if (abs(LinearVelocity)  < 0.001) LinearVelocity  = 0.0;
  if (abs(AngularVelocity) < 0.001) AngularVelocity = 0.0;

  double SetSpeedR = (((2*LinearVelocity)+(AngularVelocity*L))/(2*R))*R;   //M/S - линейная скорость колеса
  motorR.setVelocity(SetSpeedR);

  double SetSpeedL = (((2*LinearVelocity)-(AngularVelocity*L))/(2*R))*R;
  motorL.setVelocity(SetSpeedL);
}

void SetSpeed(double LinearVelocity, double AngularVelocity) {
  double SetSpeedR = (((2*LinearVelocity)+(AngularVelocity*L))/(2*R))*R;   //M/S - линейная скорость колеса
  motorR.setVelocity(SetSpeedR);

  double SetSpeedL = (((2*LinearVelocity)-(AngularVelocity*L))/(2*R))*R;
  motorL.setVelocity(SetSpeedL);
}

void EncoderInit() { //Initialize encoder interruption
  pinMode(encoderRpinA,INPUT);  // Right weel
  pinMode(encoderRpinB,INPUT);
  pinMode(encoderLpinA,INPUT);  // Left weel
  pinMode(encoderLpinB,INPUT);

  // Привязка прерывания по импульсу энкодера
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), WheelPulseR, RISING );
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), WheelPulseL, RISING );
}

void WheelPulseR() {   // Счетчик спиц правого колеса
  motorR.handleInterruptEncoder();
}

void WheelPulseL() {   // Счетчик спиц левого колеса
  motorL.handleInterruptEncoder();
}
