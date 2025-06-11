#ifndef PS2CONTROLLER_H
#define PS2CONTROLLER_H

#include <PS2X_lib.h>
#include "Motor.h"
#include "Servo.h"
#include "Other.h"
#include <math.h>

#define X_JOY_CALIB 127
#define Y_JOY_CALIB 128

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

#define TOP_SPEED 4095
#define NORM_SPEED 2048
#define TURNING_FACTOR 1

#define SINGLE_HAND_DRIVING 0
#define TWO_HAND_DRIVING 1

class PS2Controller {
private:
    PS2X ps2x;
    bool drivingMode;
    MotorController* motorController;
    ServoController* servoController;

public:
    PS2Controller(MotorController* motorCtrl, ServoController* servoCtrl);

    void setup();

    void toggleDrivingMode();

    int getSpeed();

    void getJoystickValues(int& nJoyX, int& nJoyY);

    void calculateMotorMix(int nJoyX, int nJoyY, int& nMotMixL, int& nMotMixR);

    void setMotorSpeeds(int nMotMixL, int nMotMixR, int speed);

    void controlMotors();

    void controlServos();

    void control();
}

#endif // PS2CONTROLLER_H