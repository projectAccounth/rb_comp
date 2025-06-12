#include "Other.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

namespace Utils {
    // Function to blink an LED
    void blinkLED(uint8_t times, uint16_t delayMs) {
        uint8_t pin = 13;
        pinMode(pin, OUTPUT);
        for (uint8_t i = 0; i < times; i++) {
            digitalWrite(pin, HIGH);
            delay(delayMs);
            digitalWrite(pin, LOW);
            delay(delayMs);
        }
    }

    // Function to map a value with constraints
    int constrainedMap(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
        value = constrain(value, fromLow, fromHigh);
        return map(value, fromLow, fromHigh, toLow, toHigh);
    }

    // Function to print debug messages
    void debugPrint(const char* message) {
        Serial.println(message);
    }
}