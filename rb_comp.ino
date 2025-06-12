#include "PS2Controller.h"

// Constants
const uint16_t PWM_FREQ = 50;
const uint16_t LOOP_DELAY_MS = 10;

// Initialize hardware controllers
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MotorController motorController = MotorController(&pwm);
ServoController servoController = ServoController(&pwm);
PS2Controller ps2Controller = PS2Controller(&motorController, &servoController);

TaskHandle_t straightBackwardTaskHandle = NULL;
TaskHandle_t turningTaskHandle = NULL;
TaskHandle_t brakingTaskHandle = NULL;
TaskHandle_t othersTaskHandle = NULL;

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

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(handleStraightBackward, "StraightBackwardTask", 2048, NULL, 1, &straightBackwardTaskHandle, 0);
    xTaskCreatePinnedToCore(handleTurning, "TurningTask", 2048, NULL, 1, &turningTaskHandle, 0);
    xTaskCreatePinnedToCore(handleBraking, "BrakingTask", 2048, NULL, 1, &brakingTaskHandle, 0);
    xTaskCreatePinnedToCore(handleOthers, "OthersTask", 2048, NULL, 1, &othersTaskHandle, 0);
    Serial.println("Tasks created. Entering main loop.");
}

void loop() {
    ps2Controller.ps2x.read_gamepad(false, false);
    ps2Controller.controlServos();

    int nJoyX, nJoyY;
    ps2Controller.getJoystickValues(nJoyX, nJoyY);

    delay(LOOP_DELAY_MS); // adjust as needed for responsiveness
}

void handleStraightBackward(void* parameter) {
    while (true) {
        ps2Controller.ps2x.read_gamepad(false, false);
        if (ps2Controller.ps2x.Button(PSB_PAD_UP)) {
            ps2Controller.goStraight();
        } else if (ps2Controller.ps2x.Button(PSB_PAD_DOWN)) {
            ps2Controller.goBackward();
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void handleTurning(void* parameter) {
    while (true) {
        ps2Controller.ps2x.read_gamepad(false, false);
        if (ps2Controller.ps2x.Button(PSB_PAD_LEFT)) {
            ps2Controller.goLeft();
        } else if (ps2Controller.ps2x.Button(PSB_PAD_RIGHT)) {
            ps2Controller.goRight();
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void handleBraking(void* parameter) {
    while (true) {
        ps2Controller.ps2x.read_gamepad(false, false);
        if (ps2Controller.ps2x.Button(PSB_L2)) {
            ps2Controller.brake();
        }
        vTaskDelay(40 / portTICK_PERIOD_MS);
    }
}

void handleOthers(void* parameter) {
    while (true) {
        ps2Controller.ps2x.read_gamepad(false, false);
        if (ps2Controller.ps2x.ButtonPressed(PSB_SELECT)) {
            ps2Controller.toggleDrivingMode();
        }
        if (ps2Controller.ps2x.ButtonPressed(PSB_START)) {
            ps2Controller.brake();
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}