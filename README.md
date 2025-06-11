# controls

## motor
### `PSB_PAD_UP`
- What this does: Motor 2 and Motor 3 rotate forward.
- eff:
  - Motor 2 speed set to `2000` (forward).
  - Motor 3 speed set to `2000` (forward).

### `PSB_PAD_DOWN`
- What this does: Motor 2 and Motor 3 rotate backward.
- eff:
  - Motor 2 speed set to `-2000` (backward).
  - Motor 3 speed set to `-2000` (backward).

### `PSB_PAD_LEFT`
- What this does: Motor 2 rotates backward, Motor 3 rotates forward (turn left).
- eff:
  - Motor 2 speed set to `-2000` (backward).
  - Motor 3 speed set to `2000` (forward).

### `PSB_PAD_RIGHT`
- What this does: Motor 2 rotates forward, Motor 3 rotates backward (turn right).
- eff:
  - Motor 2 speed set to `2000` (forward).
  - Motor 3 speed set to `-2000` (backward).

---

## servo
### `PSB_L1`
- What this does: Servo 1 rotates counter-clockwise.
- eff:
  - Servo 1 pulse width decreased by `100`.

### `PSB_R1`
- What this does: Servo 1 rotates clockwise.
- eff:
  - Servo 1 pulse width increased by `100`.

### `PSB_SQUARE`
- What this does: Decreases servo speed.
- eff:
  - Servo speed decreased by `100`, with a minimum threshold of `1000`.

### `PSB_CIRCLE`
- What this does: Increases servo speed.
- eff:
  - Servo speed increased by `100`, with a maximum threshold of `2000`.

---

## notes
- **Driving Mode Toggle**:
  - Controlled by `PSB_SELECT`.
  - Toggles between single-hand and two-hand driving modes.
  - Feedback: Prints "Driving mode toggled to: [current mode]" to the serial monitor.

- **Speed Control**:
  - Controlled by `PSB_R2`.
  - Sets motor speed to `TOP_SPEED` (4095) when pressed, defaults to `NORM_SPEED` (2048) otherwise.

---

## example (in case i forget)
### motor
- Press `Pad Up` to move Motor 2 and Motor 3 forward.
- Press `Pad Down` to move Motor 2 and Motor 3 backward.
- Press `Pad Left` or `Pad Right` to turn.

### servo
- Press `L1` or `R1` to rotate Servo 1 counter-clockwise or clockwise.
- Press `Square` or `Circle` to decrease or increase servo speed.

---
