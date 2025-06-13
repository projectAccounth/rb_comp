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
    MotorController* motorController;
    ServoController* servoController;
    PS2X ps2x;

    Direction currentDirection = Direction::NONE;
    Direction requestedDirection = Direction::NONE;

    SemaphoreHandle_t directionMutex;

    const int maxPWM = 4000;
    const int pwmIncrement = 75;
    int currentPWM = 0;
    bool braking = false;

    void stopMotors();
    void moveStep(Direction dir);
    void brake(Direction dir, int startPWM);
    void goStraight();
    void goBackward();
    void goLeft();
    void goRight();

    void controlServos();

    static void inputTask(void* pv);
    static void movementTask(void* pv);
    static void brakeTask(void* pv);
    static void servoTask(void* pv);
    static void controlMotor1(void* pv);
    
    void getJoystickValues(int& nJoyX, int& nJoyY);
    void calculateMotorMix(int nJoyX, int nJoyY, int& nMotMixL, int& nMotMixR);
    void setMotorSpeeds(int nMotMixL, int nMotMixR, int speed);

public:

    PS2Controller(MotorController* motorCtrl, ServoController* servoCtrl);

    void test();
    void setup();
    void toggleDrivingMode();
};

#endif // PS2CONTROLLER_H