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

enum class Direction {
    NONE, FORWARD, BACKWARD, RIGHT, LEFT
};

class PS2Controller {
private:
    bool drivingMode;
    MotorController* motorController;
    ServoController* servoController;

    const int topSpeedCap = 4000;
    const int accelIncrement = 75;
    const int actionDelay = 20; // Delay for motor actions in milliseconds
public:
    Direction currentDirection = Direction::NONE;
    PS2X ps2x;

    PS2Controller(MotorController* motorCtrl, ServoController* servoCtrl);

    void setup();

    void toggleDrivingMode();

    int getSpeed();

    void getJoystickValues(int& nJoyX, int& nJoyY);

    void calculateMotorMix(int nJoyX, int nJoyY, int& nMotMixL, int& nMotMixR);

    void setMotorSpeeds(int nMotMixL, int nMotMixR, int speed);

    void goStraight();
    void goBackward();
    void goLeft();
    void goRight();
    void brake();

    void controlServos();

    void getJoystickValues(int& nJoyX, int& nJoyY);
};

#endif // PS2CONTROLLER_H