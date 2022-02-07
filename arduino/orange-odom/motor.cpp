#include "motor.h"

Motor::Motor(const Motor::MotorPins& mp, const Motor::PIDcoefs& pid, const Motor::MotorParams& params) {
  this->motorPins = mp;

  pinMode(this->motorPins.pinDir, OUTPUT);
  pinMode(this->motorPins.pinPWM, OUTPUT);

  digitalWrite(this->motorPins.pinDir, LOW);
  analogWrite(this->motorPins.pinPWM, 0);

  this->motorParams = params;

  this->PIDcontroller = new PID(&this->realSpeedPID, &this->outputSpeedPID, &this->targetSpeedPID,
              pid.P, pid.I, pid.D, DIRECT);
  this->PIDcontroller->SetMode(AUTOMATIC);
  this->PIDcontroller->SetOutputLimits(0,255);
  this->PIDcontroller->SetSampleTime(50);
}

void Motor::update() {
  if (micros() - this->timeLastInterrupt > 100000) {
    this->realSpeed = 0;
    this->isMoving = false;
  }

  this->targetSpeedPID = this->targetSpeed * 255 / this->motorParams.maxSpeed;  // уставка скорости
  this->realSpeedPID = this->realSpeed * 255 / this->motorParams.maxSpeed;  // обратная связь ПИД-регулятора, м/сек

  this->PIDcontroller->Compute();

  if (abs(this->targetSpeed) < 0.01) {
    analogWrite(this->motorPins.pinPWM, 0);
  } else {
    analogWrite(this->motorPins.pinPWM, this->outputSpeedPID > 13 ? this->outputSpeedPID : 0);
  }
  digitalWrite(this->motorPins.pinDir, !this->isForward);
}

void Motor::handleInterruptEncoder() {
  if (this->isMoving) {
    this->realSpeed = ((2 * 3.14 * this->motorParams.wheelRadius) / this->motorParams.PPR) / double(micros() - this->timeLastInterrupt) * 1000000;
  } else {
    this->realSpeed = 0;
    this->isMoving = true;
  }

  this->timeLastInterrupt = micros();
}

void Motor::setVelocity(double speed) {
  this->targetSpeed = abs(speed);
  this->isForward = speed > 0;
}

double Motor::getVelocity() {
  return (this->isForward ? 1 : -1) * this->realSpeed;
}
