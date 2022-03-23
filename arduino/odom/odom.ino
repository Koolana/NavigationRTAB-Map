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

#include "configs/blue.h"
// #include "configs/orange.h"

//Encoder variables
const byte encoderRpinA = ENCODER_R_A;                              //A pin -> the interrupt pin (2)
const byte encoderRpinB = ENCODER_R_B;                              //B pin -> the digital pin (16)
const byte encoderLpinA = ENCODER_L_A;                              //A pin -> the interrupt pin (3)
const byte encoderLpinB = ENCODER_L_B;                              //B pin -> the digital pin (17)

// Timer variables
const double rate = 30;  // Hz
const double dT = (1 / rate);  // 100 ms = 10 times per sec - Timer interrupt interval
const unsigned long Timer1Interval = long(dT * 1000000);  // период счета

//Motor control variables
const int MotorRdir = MOTOR_R_DIR;  // 4  //Right motor Direction Control pin
const int MotorLdir = MOTOR_L_DIR;  // 7  //Left motor Direction Control pin
const int MotorRpwm = MOTOR_R_PWM;  // 5   //Right motor PWM Speed Control pin
const int MotorLpwm = MOTOR_L_PWM;  // 6   //Left motor PWM Speed Control pin

double R = WHEEL_RADIUS;
double L = BASE_WIDTH;

// максимальная линейная скорость при скважности 100%, в м/с
// unsigned int PPR = 663;  // импульсов энкодера за оборот

// double CL = 1;  // 0.939809;  // корректирующие коеффициенты для радиусов колес
// double CR = 1;  // 1.06019;

unsigned long prevTimeMicros = 0;

Motor::MotorPins motorPinsR = {MotorRdir, MotorRpwm};
Motor::MotorPins motorPinsL = {MotorLdir, MotorLpwm};
Motor::PIDcoefs pid = {P, I, D};
Motor::MotorParams paramsR = {MAX_SPEED, R, CR, PPR};
Motor::MotorParams paramsL = {MAX_SPEED, R, CL, PPR};

ros::NodeHandle nh;

Motor motorR(motorPinsR, pid, paramsR);
Motor motorL(motorPinsL, pid, paramsL);

Odometer odometer(L /*, dT */);

void cmd_velCallback(const geometry_msgs::Twist& toggle_msg);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", &cmd_velCallback);

void update_poseCallback(const geometry_msgs::PoseStamped& new_pose);
ros::Subscriber<geometry_msgs::PoseStamped> sub_update_pose("/updated_pose", &update_poseCallback);

void setup() {
  nh.initNode();

  odometer.setupPublisher(nh);
  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_update_pose);

  EncoderInit();
}

void loop() {
  motorR.update();
  motorL.update();

  if (abs(micros() - prevTimeMicros) > Timer1Interval) {
    odometer.updateByWDistance(motorL.getMovedDistance(), motorR.getMovedDistance());
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

void update_poseCallback(const geometry_msgs::PoseStamped& new_pose) {
  odometer.updateByWDistance(motorL.getMovedDistance(), motorR.getMovedDistance());
  odometer.publish(nh.now());
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
