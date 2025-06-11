#include "Servo.h"
#include "Other.h"

ServoController::ServoController(uint8_t ledPin) : notifyLED(ledPin) {
    // Map ports to channels (1st port = channel 2, 6th port = channel 7)
    for (uint8_t i = 0; i < 6; i++) {
        portChannels[i] = 2 + i;
    }
}

void ServoController::init() {
    pwm.begin();
    pwm.setPWMFreq(50); // Set frequency to 50Hz for servos
    pinMode(notifyLED, OUTPUT);
}

void ServoController::setServoByPort(uint8_t port, uint16_t pulse) {
    if (port < 1 || port > 6) {
        Serial.println("Invalid port number! Must be between 1 and 6.");
        return;
    }
    uint8_t channel = portChannels[port - 1];
    pwm.setPWM(channel, 0, pulse);
}

void ServoController::testServo(uint16_t step, uint16_t delayMs) {
    Serial.println("Servo test started...");

    // Blink LED to notify the test is starting
    Utils::blinkLED(notifyLED, 3, 500);

    for (uint8_t port = 1; port <= 6; port++) {
        uint8_t channel = portChannels[port - 1];

        // Set servo to 0 before test
        setServoByPort(port, 0);

        // Sweep from 0 to maximum pulse width in steps
        for (uint16_t pulse = 0; pulse <= 4000; pulse += step) {
            Serial.print("Setting servo on port ");
            Serial.print(port);
            Serial.print(" (channel ");
            Serial.print(channel);
            Serial.print(") to pulse ");
            Serial.println(pulse);
            setServoByPort(port, pulse);
            delay(delayMs);
        }

        // Set servo to 0 after test
        setServoByPort(port, 0);
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