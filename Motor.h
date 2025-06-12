#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <stdint.h>
#include "Other.h"

#define MIN_PWM 0
#define MAX_PWM 30

class MotorController {
private:
    Adafruit_PWMServoDriver* pwm;
    int motorChannels[8]; // Array to map motor channels (8 to 15)

public:
    MotorController(Adafruit_PWMServoDriver* pwm);

    void init();

    void setMotorSpeed(int motorIndex, int speed);

    void setAllMotorSpeeds(int speed1, int speed2, int speed3, int speed4);

    void testMotorsSequentially(uint16_t step, uint16_t delayMs);

    void testMotorsSimultaneously(uint16_t delayMs);
};

#endif // MOTOR_CONTROLLER_H