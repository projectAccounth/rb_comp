#include "Motor.h"

MotorController::MotorController(Adafruit_PWMServoDriver* pwm): pwm(pwm) {
    // Map motor channels (8 to 15)
    for (int i = 0; i < 8; i++) {
        motorChannels[i] = 8 + i;
    }
}

void MotorController::init() {
    Serial.println("Motor controller initialized");
}

void MotorController::setMotorSpeed(int motorIndex, int speed, bool inverted) {
    if (motorIndex < 0 || motorIndex >= 4) {
        Serial.println("Invalid motor index! Must be between 0 and 3.");
        return;
    }

    int channelA = motorChannels[motorIndex * 2];
    int channelB = motorChannels[motorIndex * 2 + 1];

    if (speed >= 0) {
        pwm->setPin(channelA, speed, inverted);
        pwm->setPin(channelB, 0, inverted);
    } else {
        pwm->setPin(channelA, 0, inverted);
        pwm->setPin(channelB, abs(speed), inverted);
    }
}

void MotorController::setAllMotorSpeeds(int speed1, int speed2, int speed3, int speed4) {
    setMotorSpeed(0, speed1);
    setMotorSpeed(1, speed2);
    setMotorSpeed(2, speed3);
    setMotorSpeed(3, speed4);
}

void MotorController::setMotorWheelSpeed(int speedL, int speedR) {
    setMotorSpeed(1, speedL);
    setMotorSpeed(2, speedR);
}

void MotorController::testMotorsSequentially(uint16_t step, uint16_t delayMs) {
    Serial.println("Starting sequential motor test...");

    Utils::blinkLED(3, 500);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) pwm->setPin(motorChannels[j], 0); // Reset all motors

        for (int pwm_val = 0; pwm_val <= MAX_PWM; pwm_val += 100) {
            Serial.print("Motor channel ");
            Serial.print(motorChannels[i]);
            Serial.print(" running at ");
            Serial.println(pwm_val);
            pwm->setPin(motorChannels[i], pwm_val);
            delay(delayMs);
        }

        pwm->setPin(motorChannels[i], 0); // Reset motor after test
    }
}

void MotorController::testMotorsSimultaneously(uint16_t delayMs) {
    Serial.println("Starting simultaneous motor test...");

    Utils::blinkLED(1, 500);

    Serial.println("Motor 1 & 3 forward, Motor 2 & 4 backward at 2000 for 3 seconds");
    setAllMotorSpeeds(MAX_PWM, -MAX_PWM, MAX_PWM, -MAX_PWM);
    delay(delayMs);

    Serial.println("All motors stop for 0.5 seconds");
    setAllMotorSpeeds(0, 0, 0, 0);
    delay(500);

    Serial.println("Motor 1 & 3 backward, Motor 2 & 4 forward at 3500 for 3 seconds");
    setAllMotorSpeeds(-MAX_PWM, MAX_PWM, -MAX_PWM, MAX_PWM);
    delay(delayMs);

    Serial.println("All motors stop");
    setAllMotorSpeeds(0, 0, 0, 0);
}