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

void PS2Controller::goStraight() {
    currentDirection = Direction::FORWARD;
    Serial.println("Going straight.");
    for (int i = 0; i < topSpeedCap; i += accelIncrement) {
        motorController->setAllMotorSpeeds(0, i, i, 0); // 2 and 3 forward, 1 and 4 for other purposes
        delay(actionDelay);
    }
}

void PS2Controller::goBackward() {
    currentDirection = Direction::BACKWARD;
    Serial.println("Going back.");
    for (int i = 0; i < topSpeedCap; i += accelIncrement) {
        motorController->setAllMotorSpeeds(0, -i, -i, 0); // 2 and 3 forward, 1 and 4 for other purposes
        delay(actionDelay);
    }
}

void PS2Controller::goLeft() {
    currentDirection = Direction::LEFT;
    Serial.println("Going left.");
    for (int i = 0; i < topSpeedCap; i += accelIncrement) {
        motorController->setAllMotorSpeeds(0, -i, i, 0); // 2 and 3 forward, 1 and 4 for other purposes
        delay(actionDelay);
    }
}

void PS2Controller::goRight() {
    currentDirection = Direction::LEFT;
    Serial.println("Going right.");
    for (int i = 0; i < topSpeedCap; i += accelIncrement) {
        motorController->setAllMotorSpeeds(0, i, -i, 0); // 2 and 3 forward, 1 and 4 for other purposes
        delay(actionDelay);
    }
}

void PS2Controller::brake() {
    currentDirection = Direction::LEFT;
    Serial.println("Going straight.");
    for (int i = 0; i < topSpeedCap / 2; i += accelIncrement) {
        switch (currentDirection) {
            case Direction::FORWARD:
                motorController->setAllMotorSpeeds(0, -i, -i, 0);
                break;
            case Direction::BACKWARD:
                motorController->setAllMotorSpeeds(0, i, i, 0);
                break;
            case Direction::LEFT:
                motorController->setAllMotorSpeeds(0, i, -i, 0);
                break;
            case Direction::RIGHT:
                motorController->setAllMotorSpeeds(0, -i, i, 0);
                break;
            default: break;
        }
        delay(actionDelay);
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

void PS2Controller::getJoystickValues(int& nJoyX, int& nJoyY) {
    nJoyX = ps2x.Analog(PSS_LX) - X_JOY_CALIB;
    nJoyY = ps2x.Analog(PSS_LY) - Y_JOY_CALIB;
}