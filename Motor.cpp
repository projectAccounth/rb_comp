#include "Motor.h"

MotorController::MotorController(uint8_t ledPin) : notifyLED(ledPin) {
    // Map motor channels (8 to 15)
    for (int i = 0; i < 8; i++) {
        motorChannels[i] = 8 + i;
    }
}

void MotorController::init() {
    pwm.begin();
    pwm.setPWMFreq(50); // Set frequency to 50Hz for motors
    pinMode(notifyLED, OUTPUT);
}

void MotorController::setMotorSpeed(int motorIndex, int speed) {
    if (motorIndex < 0 || motorIndex >= 8) {
        Serial.println("Invalid motor index! Must be between 0 and 7.");
        return;
    }

    int channelA = motorChannels[motorIndex * 2];
    int channelB = motorChannels[motorIndex * 2 + 1];

    if (speed >= 0) {
        pwm.setPin(channelA, speed);
        pwm.setPin(channelB, 0);
    } else {
        pwm.setPin(channelA, 0);
        pwm.setPin(channelB, abs(speed));
    }
}

void MotorController::setAllMotorSpeeds(int speed1, int speed2, int speed3, int speed4) {
    setMotorSpeed(0, speed1);
    setMotorSpeed(1, speed2);
    setMotorSpeed(2, speed3);
    setMotorSpeed(3, speed4);
}

void MotorController::testMotorsSequentially(uint16_t step, uint16_t delayMs) {
    Serial.println("Starting sequential motor test...");

    Utils::blinkLED(notifyLED, 3, 500);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) pwm.setPin(motorChannels[j], 0); // Reset all motors

        for (int pwm_val = 0; pwm_val <= MAX_PWM; pwm_val += step) {
            Serial.print("Motor channel ");
            Serial.print(motorChannels[i]);
            Serial.print(" running at ");
            Serial.println(pwm_val);
            pwm.setPin(motorChannels[i], pwm_val);
            delay(delayMs);
        }

        pwm.setPin(motorChannels[i], 0); // Reset motor after test
    }
}

void MotorController::testMotorsSimultaneously(uint16_t delayMs) {
    Serial.println("Starting simultaneous motor test...");

    Utils::blinkLED(notifyLED, 1, 500);

    Serial.println("Motor 1 & 3 forward, Motor 2 & 4 backward at 2000 for 3 seconds");
    setAllMotorSpeeds(2000, -2000, 2000, -2000);
    delay(delayMs);

    Serial.println("All motors stop for 0.5 seconds");
    setAllMotorSpeeds(0, 0, 0, 0);
    delay(500);

    Serial.println("Motor 1 & 3 backward, Motor 2 & 4 forward at 3500 for 3 seconds");
    setAllMotorSpeeds(-3500, 3500, -3500, 3500);
    delay(delayMs);

    Serial.println("All motors stop");
    setAllMotorSpeeds(0, 0, 0, 0);
}