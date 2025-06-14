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

    CONTROLS["FORWARD"] = PSB_PAD_UP;
    CONTROLS["REVERSE"] = PSB_PAD_DOWN;
    CONTROLS["LEFT"] = PSB_PAD_LEFT;
    CONTROLS["RIGHT"] = PSB_PAD_RIGHT;
    CONTROLS["BRAKE"] = PSB_L1;
    CONTROLS["TOGGLE_GAPPLE"] = PSB_SQUARE;

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
        controller->braking = controller->ps2x.ButtonPressed(controller->CONTROLS.at("BRAKE"));

        if (controller->ps2x.ButtonPressed(controller->CONTROLS.at("BRAKE"))) {
            controller->brakingRequested = true;
            controller->requestedDirection = Direction::NONE;  // cancel driving input immediately
        } else if (controller->ps2x.Button(controller->CONTROLS.at("FORWARD"))) {
            controller->requestedDirection = Direction::FORWARD;
        } else if (controller->ps2x.Button(controller->CONTROLS.at("REVERSE"))) {
            controller->requestedDirection = Direction::BACKWARD;
        } else if (controller->ps2x.Button(controller->CONTROLS.at("LEFT"))) {
            controller->requestedDirection = Direction::LEFT;
        } else if (controller->ps2x.Button(controller->CONTROLS.at("RIGHT"))) {
            controller->requestedDirection = Direction::RIGHT;
        } else if (controller->ps2x.Button(controller->CONTROLS.at("TOGGLE_GAPPLE"))) {
            controller->toggleServoPulse();
        } else {
            controller->requestedDirection = Direction::NONE;
        }

        xSemaphoreGive(controller->directionMutex);
        vTaskDelay(pdMS_TO_TICKS(10));
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

void PS2Controller::brakeTask(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);
    const int holdingPWM = 30;

    while (true) {
        bool softBrakeNow = controller->brakingRequested;
        bool holdingBrake = controller->braking;

        Direction dir = Direction::NONE;
        int pwm = 0;

        if (softBrakeNow) {
            // Get and clear state under mutex
            xSemaphoreTake(controller->directionMutex, portMAX_DELAY);
            dir = controller->currentDirection;
            pwm = controller->currentPWM;
            controller->brakingRequested = false;
            controller->currentDirection = Direction::NONE;
            controller->currentPWM = 0;
            xSemaphoreGive(controller->directionMutex);

            if (dir != Direction::NONE) {
                controller->brake(dir, pwm);  // Soft brake once
            }
        }

        // Apply holding brake as long as button is held (doesn't seem to do what it is supposed to)
        if (holdingBrake) {
            // Get current direction
            xSemaphoreTake(controller->directionMutex, portMAX_DELAY);
            dir = controller->currentDirection;
            xSemaphoreGive(controller->directionMutex);

            if (dir != Direction::NONE) {
                controller->applyHoldingBrake(dir, holdingPWM);
            }
        } else {
            controller->stopMotors();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void PS2Controller::applyHoldingBrake(Direction lastDir, int holdPWM) {
    int leftPWM = 0, rightPWM = 0;

    switch (lastDir) {
        case Direction::FORWARD:
            leftPWM = holdPWM;
            rightPWM = holdPWM;
            break;
        case Direction::BACKWARD:
            leftPWM = -holdPWM;
            rightPWM = -holdPWM;
            break;
        case Direction::LEFT:
            leftPWM = -holdPWM;
            rightPWM = holdPWM;
            break;
        case Direction::RIGHT:
            leftPWM = holdPWM;
            rightPWM = -holdPWM;
            break;
        default:
            return;  // Don't apply holding brake if we don't know the last direction
    }

    motorController->setMotorWheelSpeed(leftPWM, rightPWM);
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

    const static float curveFactor = 0.6;

    while (true) {
        // Check if the requested direction has changed
        xSemaphoreTake(directionMutex, portMAX_DELAY);
        Direction latestRequest = requestedDirection;
        bool shouldExit = (latestRequest != currentDirection || braking);
        xSemaphoreGive(directionMutex);
        if (shouldExit) break;

        bool isTurning = currentDirection == Direction::LEFT || currentDirection == Direction::RIGHT;

        int maximumReachablePWM = isTurning ? maxPWM * 0.5 : maxPWM;
        int pwmRamp = isTurning ? pwmIncrement * 1.5 : pwmIncrement * 1.3;

        // Increase PWM linearly, clamp to max
        currentPWM = std::min(currentPWM + pwmRamp, maximumReachablePWM);

        // Apply quadratic curve
        float scaled = pow(float(currentPWM) / float(maximumReachablePWM), curveFactor);
        int adjustedPWM = int(scaled * scaled * maximumReachablePWM);

        // Drive motors
        int leftPWM = adjustedPWM;
        int rightPWM = adjustedPWM;

        switch (currentDirection) {
            case Direction::FORWARD:
                break; // keep both full
            case Direction::BACKWARD:
                leftPWM = -leftPWM;
                rightPWM = -rightPWM;
                break;
            case Direction::LEFT:
                rightPWM = -rightPWM; // Reduce inner wheel speed (basically just inverted it)
                break;
            case Direction::RIGHT:
                leftPWM = -leftPWM;
                break;
            default:
                break;
        }

        motorController->setMotorWheelSpeed(leftPWM, rightPWM);

        vTaskDelay(pdMS_TO_TICKS(50));
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
    const int delayMs = 30;
    const float reverseTorqueRatio = 0.25f;  // 25% reverse torque
    const bool useCounterTorque = true;

    for (int i = steps; i >= 1; --i) {
        int brakePWM = (initialPWM * i) / steps;

        int leftPWM = 0, rightPWM = 0;
        switch (dir) {
            case Direction::FORWARD:
                leftPWM = -brakePWM;
                rightPWM = -brakePWM;
                break;
            case Direction::BACKWARD:
                leftPWM = brakePWM;
                rightPWM = brakePWM;
                break;
            case Direction::LEFT:
                leftPWM = -brakePWM;
                rightPWM = brakePWM;
                break;
            case Direction::RIGHT:
                leftPWM = brakePWM;
                rightPWM = -brakePWM;
                break;
            default:
                break;
        }

        motorController->setMotorWheelSpeed(leftPWM, rightPWM);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }

    // apply a brief counter-torque to cancel momentum
    if (useCounterTorque && dir != Direction::NONE) {
        int reversePWM = int(initialPWM * reverseTorqueRatio);
        int leftPWM = 0, rightPWM = 0;

        switch (dir) {
            case Direction::FORWARD:
                leftPWM = reversePWM;
                rightPWM = reversePWM;
                break;
            case Direction::BACKWARD:
                leftPWM = -reversePWM;
                rightPWM = -reversePWM;
                break;
            case Direction::LEFT:
                leftPWM = reversePWM;
                rightPWM = -reversePWM;
                break;
            case Direction::RIGHT:
                leftPWM = -reversePWM;
                rightPWM = reversePWM;
                break;
            default:
                break;
        }

        motorController->setMotorWheelSpeed(leftPWM, rightPWM);
        vTaskDelay(pdMS_TO_TICKS(60));  // brief counter impulse
    }

    stopMotors();  // Finally hold position
}

void PS2Controller::servoTask(void* pv) {
    PS2Controller* controller = static_cast<PS2Controller*>(pv);

    const int analogCenter = 128;
    const int deadzone = 10;

    const float centerAngle = 90.0f;
    const float angleRange = 45.0f;  // maximum deviation from center
    const float degreesPerSecond = 300.0f;  // max servo rotation speed
    const int tickMs = 50;
    const float maxStepPerTick = (degreesPerSecond * tickMs) / 1000.0f;

    const bool resetToCenterWhenReleased = true;

    float currentAngle = centerAngle;
    float targetAngle = centerAngle;

    controller->servoController->setServoByPort(1, currentAngle);

    while (true) {
        int ly = controller->ps2x.Analog(PSS_LY);

        // Handle deadzone and joystick input
        if (abs(ly - analogCenter) < deadzone * 2) {
            if (resetToCenterWhenReleased) {
                targetAngle = centerAngle;
            }
            // else: maintain current targetAngle
        } else {
            float offset = float(ly - analogCenter) / 127.0f;  // range [-1, 1]
            targetAngle = std::clamp(centerAngle + offset * angleRange, 0.0f, 180.0f);
        }

        // Smooth movement toward targetAngle based on speed per tick
        float delta = targetAngle - currentAngle;
        if (abs(delta) > 0.1f) {
            float step = std::clamp(delta, -maxStepPerTick, maxStepPerTick);
            currentAngle += step;
            controller->servoController->setServoByPort(1, currentAngle);
        }

        vTaskDelay(pdMS_TO_TICKS(tickMs));
    }
}

void PS2Controller::toggleServoPulse() {
    const uint8_t port = 6;
    const float centerAngle = 90.0f;
    const float maxAngle = 180.0f;
    const float stepAngle = 5.0f;
    const int delayMs = 30;

    static bool toggled = false;

    if (!toggled) {
        // rotate clockwise to maxAngle
        for (float angle = centerAngle; angle <= maxAngle; angle += stepAngle) {
            servoController->setServoByPort(port, angle);
            vTaskDelay(pdMS_TO_TICKS(delayMs));
        }
        toggled = true;
    } else {
        // rotate counterclockwise back to center
        for (float angle = maxAngle; angle >= centerAngle; angle -= stepAngle) {
            servoController->setServoByPort(port, angle);
            vTaskDelay(pdMS_TO_TICKS(delayMs));
        }
        toggled = false;
    }
}

void PS2Controller::test() {
    motorController->testMotorsSequentially(100, 50);
    servoController->testServo(100, 50);
}