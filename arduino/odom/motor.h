#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <PID_v1.h>
#include <TimerOne.h>

class Motor {
  public:
    struct MotorPins {
      uint8_t pinDir;
      uint8_t pinPWM;
    };

    struct PIDcoefs {
      double P;
      double I;
      double D;
    };

    struct MotorParams {
      double maxSpeed;
      double wheelRadius;
      double corrFactor;
      unsigned int PPR;
    };

    Motor(const Motor::MotorPins& mp, const Motor::PIDcoefs& pid, const Motor::MotorParams& params);

    void update();
    void handleInterruptEncoder();

    double getVelocity();
    void setVelocity(double speed);
    double getMovedDistance();

  private:
    Motor::MotorPins motorPins;
    Motor::MotorParams motorParams;

    PID* PIDcontroller;

    unsigned long timeLastInterrupt = 0;
    bool isMoving = false;

    double targetSpeed = 0;  // speed abs(m/sec)
    double isForward = true;

    double realSpeed = 0;  // speed (m/sec)

    double targetSpeedPID = 0;  // speed for PID
    double realSpeedPID = 0;  // speed for PID
    double outputSpeedPID = 0;  // speed for PID

    unsigned long numPulse = 0;
};

#endif  /* MOTOR_H */
