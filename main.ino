#include "Motor.h"
#include "Servo.h"
#include "PS2Controller.h"

MotorController motorController(0, 1, 2, 3);
PS2Controller ps2Controller(&motorController);
ServoController servoController(4);

void setup() {
    Serial.begin(115200);
    motorController.init();
    ps2Controller.setup();
    servoController.init();
}

void loop() {
    ps2Controller.control();
    delay(5); // adjust as needed for responsiveness
}