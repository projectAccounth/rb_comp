#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Other.h"
#include <stdint.h>

class ServoController {
private:
    Adafruit_PWMServoDriver pwm;
    uint8_t portChannels[6]; // Array to map ports to channels

public:
    ServoController(uint8_t ledPin);

    void init();

    void setServoByPort(uint8_t port, uint16_t pulse);

    void testServo(uint16_t step, uint16_t delayMs);

    void moveAllServos(uint16_t pulse, uint16_t delayMs);
};

#endif // SERVO_CONTROLLER_H