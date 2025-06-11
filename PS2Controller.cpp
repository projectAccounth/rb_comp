#include "PS2Controller.h"

PS2Controller::PS2Controller(MotorController* motorCtrl, ServoController* servoCtrl)
        : drivingMode(SINGLE_HAND_DRIVING), motorController(motorCtrl), servoController(servoCtrl) {}

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
    if (!drivingMode) return;
    // Pad Up and Pad Down for Motor 2 and 3 forward/backward
    if (ps2x.Button(PSB_PAD_UP)) {
        motorController->setMotorSpeed(1, 2000); // Motor 2 forward
        motorController->setMotorSpeed(2, 2000); // Motor 3 forward
        Serial.println("Motor 2 and Motor 3 running forward.");
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
        motorController->setMotorSpeed(1, -2000); // Motor 2 backward
        motorController->setMotorSpeed(2, -2000); // Motor 3 backward
        Serial.println("Motor 2 and Motor 3 running backward.");
    }

    // Pad Left and Pad Right for Motor 2 and 3 turning
    if (ps2x.Button(PSB_PAD_LEFT)) {
        motorController->setMotorSpeed(1, -2000); // Motor 2 backward
        motorController->setMotorSpeed(2, 2000);  // Motor 3 forward
        Serial.println("Motor 2 and Motor 3 turning left.");
    } else if (ps2x.Button(PSB_PAD_RIGHT)) {
        motorController->setMotorSpeed(1, 2000);  // Motor 2 forward
        motorController->setMotorSpeed(2, -2000); // Motor 3 backward
        Serial.println("Motor 2 and Motor 3 turning right.");
    }
}

void PS2Controller::controlServos() {
    static int servoSpeed = 1500; // Default servo speed

    // L1 and R1 for Servo 1 rotation
    if (ps2x.Button(PSB_L1)) {
        servoController->setServoByPort(1, servoSpeed - 100); // Rotate counter-clockwise
        Serial.println("Servo 1 rotating counter-clockwise.");
    } else if (ps2x.Button(PSB_R1)) {
        servoController->setServoByPort(1, servoSpeed + 100); // Rotate clockwise
        Serial.println("Servo 1 rotating clockwise.");
    }

    // Square (Pink) to decrease servo speed
    if (ps2x.Button(PSB_SQUARE)) {
        servoSpeed = max(1000, servoSpeed - 100); // Decrease speed, minimum threshold 1000
        Serial.print("Servo speed decreased to: ");
        Serial.println(servoSpeed);
    }

    // Circle (Red) to increase servo speed
    if (ps2x.Button(PSB_CIRCLE)) {
        servoSpeed = min(2000, servoSpeed + 100); // Increase speed, maximum threshold 2000
        Serial.print("Servo speed increased to: ");
        Serial.println(servoSpeed);
    }
}

void PS2Controller::control() {
    controlMotors();
    controlServos();
}