#include "Servo.h"
#include "Other.h"

ServoController::ServoController(Adafruit_PWMServoDriver* pwm) : pwm(pwm) {
    // Map ports to channels (1st port = channel 2, 6th port = channel 7)
    for (uint8_t i = 0; i < 6; i++) {
        portChannels[i] = 2 + i;
    }
}

void ServoController::init() {
    Serial.println("Servo controller initialized");
}

void ServoController::setServoByPort(uint8_t port, float angle) {
    if (port < 1 || port > 6) {
        Serial.println("Invalid port number! Must be between 1 and 6.");
        return;
    }

    // Constrain angle to safe range
    angle = constrain(angle, 0.0f, 180.0f);

    // Map angle (0–180°) to pulse width in µs (adjust as needed)
    const uint16_t SERVO_MIN_US = 500;   // Min pulse for 0°
    const uint16_t SERVO_MAX_US = 2500;  // Max pulse for 180°
    float pulse_us = map(angle, 0.0f, 180.0f, SERVO_MIN_US, SERVO_MAX_US);

    // Convert microseconds to PCA9685 PWM count (12-bit: 0–4095)
    uint16_t pwmVal = static_cast<uint16_t>(pulse_us * 0.2048f); // = (pulse_us / 1000000) * 50Hz * 4096

    // Send to correct channel
    uint8_t channel = portChannels[port - 1];
    pwm->setPWM(channel, 0, pwmVal);
}

void ServoController::testServo(uint16_t step, uint16_t delayMs) {
    Serial.println("Servo test started...");

    // Blink LED to notify the test is starting
    Utils::blinkLED(3, 500);

    for (uint8_t port = 1; port <= 6; port++) {
        uint8_t channel = portChannels[port - 1];

        // Set servo to 0 before test
        setServoByPort(6, 0);

        // Sweep from 0 to maximum pulse width in steps
        for (uint16_t pulse = 0; pulse <= 360; pulse += step) {
            Serial.print("Setting servo on port ");
            Serial.print(port);
            Serial.print(" (channel ");
            Serial.print(channel);
            Serial.print(") to pulse ");
            Serial.println(pulse);
            setServoByPort(6, pulse);
            delay(delayMs);
        }

        // Set servo to 0 after test
        setServoByPort(6, 0);
    }

    Serial.println("Servo test completed.");
}

void ServoController::moveAllServos(uint16_t pulse, uint16_t delayMs) {
    Serial.println("Moving all servos...");
    for (uint8_t port = 1; port <= 6; port++) {
        setServoByPort(port, pulse);
        delay(delayMs);
    }
}