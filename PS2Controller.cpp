#include "PS2Controller.h"

int vclamp(int val, int lo, int hi) {
    return min(hi, max(val, lo));
}

PS2Controller::PS2Controller(MotorController* motorCtrl, ServoController* servoCtrl)
    : motorController(motorCtrl), servoController(servoCtrl) {
    directionMutex = xSemaphoreCreateMutex();
}

void PS2Controller::setup() {
    int err = -1;
    while (err != 0) {
        err = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
        delay(1000);
        Serial.println("Connecting");
    }
    Serial.println("Connected.");

    xTaskCreatePinnedToCore(inputTask, "InputTask", 2048, this, 1, NULL, 0);
    xTaskCreatePinnedToCore(movementTask, "MovementTask", 2048, this, 1, NULL, 1);
    xTaskCreatePinnedToCore(brakeTask, "BrakeTask", 2048, this, 1, NULL, 1);
    xTaskCreatePinnedToCore(servoTask, "ServoTask", 2048, this, 1, NULL, 1);
    xTaskCreatePinnedToCore(controlMotor1, "ControlMotor1", 2048, this, 1, NULL, 1);

    delay(500);
}

void PS2Controller::controlMotor1(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);
    const int center = 128;
    const int deadzone = 10;
    static int currentPulse = 0;

    controller->motorController->setMotorSpeed(0, 0);

    while (true) {
        int lx = controller->ps2x.Analog(PSS_LX);

        if (abs(lx - center) < deadzone) {
            lx = center;
        }

        if (lx < center - deadzone * 3) { 
            currentPulse = max(currentPulse - controller->pwmIncrement, -controller->maxPWM);
        } 
        else if (lx > center + deadzone * 3) {
            currentPulse = min(currentPulse + controller->pwmIncrement, controller->maxPWM);
        }
        else {
            currentPulse = 0;
        }

        controller->motorController->setMotorSpeed(0, currentPulse);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Runs in its own task to read controller input
void PS2Controller::inputTask(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);
    while (true) {
        controller->ps2x.read_gamepad(false, false);
        xSemaphoreTake(controller->directionMutex, portMAX_DELAY);

        if (controller->ps2x.ButtonPressed(PSB_L1)) {
            controller->braking = true;
            controller->requestedDirection = Direction::NONE;  // cancel driving input immediately
        } else if (controller->ps2x.Button(PSB_PAD_UP)) {
            controller->requestedDirection = Direction::FORWARD;
        } else if (controller->ps2x.Button(PSB_PAD_DOWN)) {
            controller->requestedDirection = Direction::BACKWARD;
        } else if (controller->ps2x.Button(PSB_PAD_LEFT)) {
            controller->requestedDirection = Direction::LEFT;
        } else if (controller->ps2x.Button(PSB_PAD_RIGHT)) {
            controller->requestedDirection = Direction::RIGHT;
        } else {
            controller->requestedDirection = Direction::NONE;
        }

        xSemaphoreGive(controller->directionMutex);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Runs the main movement logic
void PS2Controller::movementTask(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);
    while (true) {
        xSemaphoreTake(controller->directionMutex, portMAX_DELAY);
        Direction dir = controller->requestedDirection;
        bool localBraking = controller->braking;
        xSemaphoreGive(controller->directionMutex);

        if (!localBraking && dir != Direction::NONE) {
            controller->moveStep(dir);
        } else if (localBraking) {
            controller->stopMotors(); // immediate halt before braking
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Runs in background and handles braking separately
void PS2Controller::brakeTask(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);
    while (true) {
        bool localBraking = false;
        Direction dir = Direction::NONE;
        int pwm = 0;

        // Copy and clear state under lock
        xSemaphoreTake(controller->directionMutex, portMAX_DELAY);
        localBraking = controller->braking;
        if (localBraking) {
            dir = controller->currentDirection;
            pwm = controller->currentPWM;
            controller->braking = false; // mark handled
            controller->currentPWM = 0;
            controller->currentDirection = Direction::NONE;
        }
        xSemaphoreGive(controller->directionMutex);

        if (localBraking && dir != Direction::NONE) {
            controller->brake(dir, pwm);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Handles smooth acceleration in a direction
void PS2Controller::moveStep(Direction dir) {
    if (dir == Direction::NONE) {
        stopMotors();
        xSemaphoreTake(directionMutex, portMAX_DELAY);
        currentPWM = 0;
        currentDirection = Direction::NONE;
        xSemaphoreGive(directionMutex);
        return;
    }

    // Update current direction
    xSemaphoreTake(directionMutex, portMAX_DELAY);
    currentDirection = dir;
    xSemaphoreGive(directionMutex);

    const static float curveFactor = 1.5;

    while (true) {
        // Check if the requested direction has changed
        xSemaphoreTake(directionMutex, portMAX_DELAY);
        Direction latestRequest = requestedDirection;
        bool shouldExit = (latestRequest != currentDirection || braking);
        xSemaphoreGive(directionMutex);
        if (shouldExit) break;

        // Increase PWM linearly, clamp to max
        currentPWM = std::min(currentPWM + pwmIncrement, maxPWM);

        // Apply quadratic curve
        float scaled = pow(float(currentPWM) / float(maxPWM), curveFactor);
        int adjustedPWM = int(scaled * scaled * maxPWM);

        // Drive motors
        switch (currentDirection) {
            case Direction::FORWARD:
                motorController->setMotorWheelSpeed(-adjustedPWM, -adjustedPWM);
                break;
            case Direction::BACKWARD:
                motorController->setMotorWheelSpeed(adjustedPWM, adjustedPWM);
                break;
            case Direction::LEFT:
                motorController->setMotorWheelSpeed(-adjustedPWM, adjustedPWM);
                break;
            case Direction::RIGHT:
                motorController->setMotorWheelSpeed(adjustedPWM, -adjustedPWM);
                break;
            default:
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Reset motor state when done
    stopMotors();
    xSemaphoreTake(directionMutex, portMAX_DELAY);
    currentPWM = 0;
    currentDirection = Direction::NONE;
    xSemaphoreGive(directionMutex);
}

void PS2Controller::stopMotors() {
    motorController->setAllMotorSpeeds(0, 0, 0, 0);
}

void PS2Controller::brake(Direction dir, int initialPWM) {
    Serial.println("Soft Braking");

    const int steps = 8;
    for (int i = steps; i >= 1; --i) {
        int brakePWM = (initialPWM * i) / steps;

        // Always preserve the original driving direction sign
        switch (dir) {
            case Direction::FORWARD:
                motorController->setMotorWheelSpeed(-brakePWM, -brakePWM);  // Forward = negative
                break;
            case Direction::BACKWARD:
                motorController->setMotorWheelSpeed(brakePWM, brakePWM);    // Backward = positive
                break;
            case Direction::LEFT:
                motorController->setMotorWheelSpeed(-brakePWM, brakePWM);
                break;
            case Direction::RIGHT:
                motorController->setMotorWheelSpeed(brakePWM, -brakePWM);
                break;
            default:
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(30));  // Longer delay = softer braking
    }

    // Stop completely after braking
    stopMotors();
}
void PS2Controller::servoTask(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);

    const int servoMin = 0;
    const int servoMax = 4095;
    const int center = 128;
    const int deadzone = 10;
    static int currentPulse = 0;

    controller->servoController->setServoByPort(1, 0);

    while (true) {
        int ly = controller->ps2x.Analog(PSS_LY);

        if (abs(ly - center) < deadzone) {
            ly = center;
        }

        if (ly < center - deadzone * 3) {
            currentPulse = max(currentPulse - controller->pwmIncrement, -servoMax);
        } else if (ly > center + deadzone * 3) {
            currentPulse = min(currentPulse + controller->pwmIncrement, servoMax);
        }

        controller->servoController->setServoByPort(1, currentPulse);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void PS2Controller::test() {
    motorController->testMotorsSequentially(100, 50);
    servoController->testServo(100, 50);
}