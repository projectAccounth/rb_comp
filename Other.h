#ifndef OTHER_H
#define OTHER_H

#include <Wire.h>
#include <math.h>
#include <stdint.h>

namespace Utils {
    // Function to blink an LED
    void blinkLED(uint8_t times, uint16_t delayMs);
    // Function to map a value with constraints
    int constrainedMap(int value, int fromLow, int fromHigh, int toLow, int toHigh);
    // Function to print debug messages
    void debugPrint(const char* message);
}

#endif // OTHER_H