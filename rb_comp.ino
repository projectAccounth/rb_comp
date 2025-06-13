#include "PS2Controller.h"

// Constants
const uint16_t PWM_FREQ = 50;
const uint16_t LOOP_DELAY_MS = 1000;

// Initialize hardware controllers
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MotorController motorController = MotorController(&pwm);
ServoController servoController = ServoController(&pwm);
PS2Controller ps2Controller = PS2Controller(&motorController, &servoController);

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    pwm.begin(); // Ensure PWM driver is initialized
    pwm.setPWMFreq(PWM_FREQ);

    motorController.init();
    servoController.init();
    ps2Controller.setup();

    delay(1000); // Allow time for setup to complete
    Serial.println("Setup complete. Ready to control.");
}

void loop() {
    // ps2Controller.testMotors();
    // Main loop can remain empty as tasks are managed by FreeRTOS
    delay(LOOP_DELAY_MS);
}