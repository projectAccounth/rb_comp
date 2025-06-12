#include "PS2Controller.h"

PS2Controller::PS2Controller(MotorController* motorCtrl, ServoController* servoCtrl)
        : drivingMode(true), motorController(motorCtrl), servoController(servoCtrl) {}

void PS2Controller::setup() {
    int err = -1;
    while (err != 0) {
        err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
    }
    Serial.println("PS2 Controller setup complete.");
}

void PS2Controller::toggleDrivingMode() {
    drivingMode = !drivingMode;
    Serial.print("Driving mode toggled to: ");
    Serial.println(drivingMode ? "Driving" : "Stopped");
}

int PS2Controller::getSpeed() {
    if (ps2x.Button(PSB_R2)) {
        return TOP_SPEED;
    }
    return NORM_SPEED;
}

void PS2Controller::controlMotors() {
    static unsigned int currentSpeed[2] = {0}; // For motor 2 and 3 (index 1 and 2)
    unsigned int targetSpeed[2] = {0};
    const unsigned int step = 200; // Speed step for smoothness
    const int delayMs = 10; // Delay for smoothness

    if (!drivingMode) {
        // Only update motors 2 and 3 if driving mode is enabled
        return;
    }

    // Determine target speeds for motors 2 and 3
    if (ps2x.Button(PSB_PAD_UP)) {
        targetSpeed[0] = (targetSpeed[0] + 1000u) % 4095; // Motor 2 backward
        targetSpeed[1] = (targetSpeed[1] + 1000u) % 4095; // Motor 3 backward
        Serial.println("Motor 2 and Motor 3 running forward.");
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
        targetSpeed[0] = (targetSpeed[0] - 1000u) % 4095; // Motor 2 backward
        targetSpeed[1] = (targetSpeed[1] - 1000u) % 4095; // Motor 3 backward
        Serial.println("Motor 2 and Motor 3 running backward.");
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
        targetSpeed[0] = (targetSpeed[0] - 1000u) % 4095; // Motor 2 backward
        targetSpeed[1] = (targetSpeed[1] + 1000u) % 4095;  // Motor 3 forward
        Serial.println("Motor 2 and Motor 3 turning left.");
    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
        targetSpeed[0] = (targetSpeed[0] + 1000u) % 4095;  // Motor 2 forward
        targetSpeed[1] = (targetSpeed[1] - 1000u) % 4095; // Motor 3 backward
        Serial.println("Motor 2 and Motor 3 turning right.");
    } else {
    }

    // Smoothly ramp currentSpeed to targetSpeed for both motors
    bool moving = true;
    while (moving) {
        moving = false;
        for (int i = 0; i < 2; i++) {
            if (currentSpeed[i] < targetSpeed[i]) {
                currentSpeed[i] = min(currentSpeed[i] + step, targetSpeed[i]);
                moving = true;
            } else if (currentSpeed[i] > targetSpeed[i]) {
                currentSpeed[i] = max(currentSpeed[i] - step, targetSpeed[i]);
                moving = true;
            }
        }
        // Apply speeds to motors 2 and 3 (index 1 and 2)
        motorController->setMotorSpeed(1, currentSpeed[0]);
        motorController->setMotorSpeed(2, currentSpeed[1]);
        if (moving) delay(200);
    }
}

void PS2Controller::controlServos() {
    static int servoSpeed = 250; // Default servo speed
    static int currentPulse[6] = {0}; // Track current pulse for each servo
    static int selectedServo = 0; // 0-based index for servo port (0=port1, 5=port6)
    int targetPulse = currentPulse[selectedServo], initialPulse = currentPulse[selectedServo];

    // Cycle selected servo with SELECT button
    if (ps2x.ButtonPressed(PSB_SELECT)) {
        selectedServo = (selectedServo + 1) % 6;
        Serial.print("Selected servo port: ");
        Serial.println(selectedServo + 1);
    }

    if (ps2x.ButtonPressed(PSB_START)) {
        servoController->setServoByPort(selectedServo + 1, 0);
        currentPulse[selectedServo] = 0;
    }

    // L1 and R1 for selected servo rotation
    if (ps2x.Button(PSB_L1)) {
        currentPulse[selectedServo] = max(0, currentPulse[selectedServo] - 100);
        targetPulse = currentPulse[selectedServo];
        Serial.print("Servo "); Serial.print(selectedServo + 1); Serial.println(" rotating counter-clockwise.");
    } else if (ps2x.Button(PSB_R1)) {
        currentPulse[selectedServo] = min(4095, currentPulse[selectedServo] + 100);
        targetPulse = currentPulse[selectedServo];
        Serial.print("Servo "); Serial.print(selectedServo + 1); Serial.println(" rotating clockwise.");
    } else {
    }

    // Square (Pink) to decrease servo speed
    if (ps2x.Button(PSB_SQUARE)) {
        servoSpeed = max(100, servoSpeed - 10);
        Serial.print("Servo speed decreased to: ");
        Serial.println(servoSpeed);
    }

    // Circle (Red) to increase servo speed
    if (ps2x.Button(PSB_CIRCLE)) {
        servoSpeed = min(2000, servoSpeed + 10);
        Serial.print("Servo speed increased to: ");
        Serial.println(servoSpeed);
    }

    // Smoothly move servo to targetPulse
    if (initialPulse < targetPulse) {
        for (int i = initialPulse; i < targetPulse; i += 10) {
            servoController->setServoByPort(selectedServo + 1, i);
            delay(20);
        }
    } else if (initialPulse > targetPulse) {
        for (int i = initialPulse; i > targetPulse; i -= 10) {
            servoController->setServoByPort(selectedServo + 1, i);
            delay(20);
        }
    }
    currentPulse[selectedServo] = targetPulse;
}
void PS2Controller::control() {
    ps2x.read_gamepad(false, false);
    controlMotors();
    controlServos();
}