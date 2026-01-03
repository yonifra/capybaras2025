# AI Agent Instructions for Capybaras2025 Robot Project

## Project Overview

This is a **LEGO SPIKE Prime v3.0 robotics project** written in **Python (MicroPython)**. The robot uses motors, sensors, and the LEGO hub to complete missions. The project is structured for a robotics competition where the robot must navigate, manipulate objects, and complete specific tasks.

## Technology Stack

- **Language**: Python (MicroPython for LEGO SPIKE Prime)
- **Hardware**: LEGO SPIKE Prime Hub
- **Key Modules**: `hub`, `motor`, `motor_pair`, `runloop`, `motion_sensor`, `button`, `time`, `math`

## Project Structure

```
capybaras2025/
├── src/
│   └── main.py          # Main robot control code
├── innovation/          # Empty directory (possibly for future innovation project submissions)
├── .gitignore          # Standard Python/IDE gitignore
└── AGENTS.md           # This file
```

## Critical Code Conventions

### 1. **Async/Await Pattern**

All robot movement and control functions MUST use `async`/`await` syntax:

```python
async def mission_example():
    await move_tank_for_cm(50, 50)
    await turn_PID(90)

# Run with runloop
runloop.run(main())
```

### 2. **Motor Configuration**

- **Left Motor**: Port D
- **Right Motor**: Port C
- **Right Arm Motor**: Port A
- **Left Arm Motor**: Port B
- **Motor Pair**: `motor_pair.PAIR_1`
- Motors are paired at startup: `motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)`

### 3. **Physical Constants**

These constants are tuned for the specific robot and MUST be verified/updated if hardware changes:

```python
WHEEL_CIRCUMFERENCE = 17.5929  # cm - For 56mm diameter wheels (small wheels)
WHEEL_BASE = 11.5              # cm - Distance between left and right wheels
CENTER_OFFSET = -4.0           # cm - Distance from wheel axle to robot's center of mass (forward)
```

### 4. **PID Constants**

PID constants are defined inside the control functions. Here are the current values:

```python
# PID Constants for turning (inside turn_PID)
kp = 2      # Proportional
ki = 0.0    # Integral
kd = 0.5    # Derivative

# PID Constants for straight driving (inside move_tank_for_cm)
kp = 1.2    # How aggressively it returns to straight
ki = 0      # Fixes slow drifts
kd = 6.66   # Prevents "fishtailing" (wobbling)
```

## Core Functions Reference

### Movement Functions

#### `cm_to_degrees(distance_cm: float) -> float`

- Converts distance in centimeters to motor degrees
- Used internally by movement functions
- Returns absolute value (always positive)

#### `move_tank_for_cm(move_cm: float, speed_per: int = 50) -> None`

- Drives robot forward/backward a specified distance with gyro-based steering correction
- `move_cm`: Distance in centimeters (positive = forward, negative = backward)
- `speed_per`: Speed as percentage (0-100%), default 50%
- **Features**:
  - PID steering correction to maintain straight line
  - Dynamic speed ramp-down near target for precision
  - Stops with `motor.HOLD` for accurate stopping

#### `turn_PID(turn_degrees: int) -> None`

- Precise point turn using PID control with gyro feedback
- `turn_degrees`: Angle to turn (positive = clockwise, negative = counter-clockwise)
- **Features**:
  - Shortest path logic (always takes the shortest rotation)
  - Acceleration ramping to prevent wheel slip
  - Safety timeout (5 seconds)
  - Settle detection (must be stable for 120ms before completing)
  - Integral windup protection

### Arm Control Functions

#### `turn_right_arm(degrees_turn: int, speed_per: int) -> None`

- Controls the right arm motor (Port A)
- `degrees_turn`: Degrees to rotate (positive/negative for direction)
- `speed_per`: Speed as percentage (0-100%)

#### `turn_left_arm(degrees_turn: int, speed_per: int) -> None`

- Controls the left arm motor (Port B)
- `degrees_turn`: Degrees to rotate (positive/negative for direction)
- `speed_per`: Speed as percentage (0-100%)

#### `oscillate_arm(arm_func, motor_degrees: int, speed: int, count: int = 4) -> None`

- Oscillates an arm motor back and forth a specified number of times
- `arm_func`: The arm function to call (`turn_right_arm` or `turn_left_arm`)
- `motor_degrees`: Degrees to move in each direction
- `speed`: Speed percentage (0-100%)
- `count`: Number of oscillation cycles (default: 4)

**Example:**

```python
# Oscillate right arm 4 times at 90 degrees, 50% speed
await oscillate_arm(turn_right_arm, 90, 50, 4)
```

## Mission Structure

Organize code into mission functions:

```python
async def mission_one_and_two():
    """Mission 1+2: Description of what this mission does."""
    print("--- Starting Mission 1+2 ---")
    await move_tank_for_cm(50, 50)  # Move forward 50cm at 50% speed
    await turn_PID(90)               # Turn 90 degrees clockwise
    await turn_right_arm(90, 50)     # Move right arm
    # Add mission-specific code here

async def main():
    await mission_one_and_two()
    await mission_three_and_four()
    await mission_eight()
    await mission_ten()
    # etc.

runloop.run(main())
```

### Current Missions

| Mission | Function                   | Description                                                                                                |
| ------- | -------------------------- | ---------------------------------------------------------------------------------------------------------- |
| 1+2     | `mission_one_and_two()`    | Navigation with arm manipulation: move forward 56cm, turn sequence, use right arm for task, return to base |
| 3+4     | `mission_three_and_four()` | Turn 90° right then 90° left                                                                               |
| 5+6     | `mission_five_and_six()`   | Navigation sequence with multiple turns                                                                    |
| 8       | `mission_eight()`          | Move forward, oscillate arm 4x, return                                                                     |
| 10      | `mission_ten()`            | Complex navigation with box manipulation                                                                   |

## Coding Guidelines for AI Agents

### DO ✅

1. **Always use async/await** for robot control functions
2. **Print debug messages** to track mission progress:
   ```python
   print("--- Starting Mission X ---")
   print(f"Moving {distance}cm at {speed}% speed")
   ```
3. **Stop motors explicitly** after movements: `motor_pair.stop(motor_pair.PAIR_1, motor.BRAKE)`
4. **Add sleep delays** when needed: `await runloop.sleep_ms(100)`
5. **Use percentage-based speeds** (0-100%) for readability
6. **Comment complex sequences** to explain mission strategy
7. **Keep missions modular** - one function per mission or sub-task
8. **Test movements incrementally** - don't chain too many actions without validation points
9. **Use module-level PID constants** - don't hardcode PID values in functions
10. **Use `oscillate_arm()` helper** - for repetitive arm movements

### DON'T ❌

1. **Don't modify constants** without physical measurements
2. **Don't use blocking operations** - always use async functions
3. **Don't forget to pair motors** at initialization
4. **Don't mix up motor ports** (D=Left, C=Right, A=Right Arm, B=Left Arm)
5. **Don't skip gyro sensor reset** in rotation functions
6. **Don't use raw motor degrees** - use `cm_to_degrees()` helper
7. **Don't assume perfect accuracy** - account for slippage and calibration errors
8. **Don't duplicate PID constants** - use module-level constants

## Available Hardware APIs

### Motion Sensor (Gyro)

```python
motion_sensor.reset_yaw(0)                    # Reset gyro to 0
current_angle = motion_sensor.tilt_angles()[0] / 10  # Get current yaw in degrees
```

### Motor Pair Control

```python
motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, degrees, left_vel, right_vel)
motor_pair.stop(motor_pair.PAIR_1, motor.BRAKE)
```

### Individual Motor Control

```python
import motor
motor.run(port.A, velocity)
await motor.run_for_degrees(port.A, degrees, velocity)
motor.stop(port.A, motor.BRAKE)
motor.reset_relative_position(port.A, 0)
motor.relative_position(port.A)  # Get current position
```

### Hub Features

```python
from hub import light_matrix, port, motion_sensor, sound
light_matrix.show_image(light_matrix.IMAGE_HAPPY)
# Add more as needed for specific missions
```

## Mission Development Workflow

1. **Understand the mission** - What needs to be accomplished?
2. **Plan the route** - Sketch movements: forward, back, turns
3. **Write mission function** - Create `async def mission_X():`
4. **Implement movements** - Use `move_tank_for_cm()` and `turn_PID()`
5. **Add attachments/manipulations** - Use `turn_right_arm()`, `turn_left_arm()`, or `oscillate_arm()`
6. **Add to main()** - Call the mission function
7. **Test and tune** - Adjust speeds, distances, and angles based on performance

## Example Mission Template

```python
async def mission_example() -> None:
    """
    Mission Description:
    - Move to target area
    - Activate mechanism
    - Return to base
    """
    print("--- Starting Mission Example ---")

    # Phase 1: Navigate to target
    await move_tank_for_cm(60, 50)      # Forward 60cm at 50% speed
    await turn_PID(45)                   # Turn 45° clockwise
    await move_tank_for_cm(30, 30)      # Approach slowly

    # Phase 2: Execute task
    await runloop.sleep_ms(500)          # Brief pause
    await turn_right_arm(90, 50)         # Activate mechanism
    await turn_right_arm(-90, 50)        # Reset mechanism

    # Phase 3: Return
    await turn_PID(-45)                  # Turn back
    await move_tank_for_cm(-60, 50)     # Reverse to base

    print("--- Mission Example Complete ---")
```

## Debugging Tips

1. **Use print statements liberally** - They appear in the console
2. **Test movements individually** before chaining
3. **Check sensor values**: Print gyro readings to verify accuracy
4. **Adjust PID constants** if movements are consistently off
5. **Add sleep delays** between rapid movements for stability
6. **Watch for wheel slippage** on different surfaces

## Port Assignments

| Port | Device | Function                                |
| ---- | ------ | --------------------------------------- |
| C    | Motor  | Right drive wheel                       |
| D    | Motor  | Left drive wheel                        |
| A    | Motor  | Right arm mechanism                     |
| B    | Motor  | Left arm mechanism                      |
| E, F | -      | Available for additional sensors/motors |

## Performance Optimization

- Use **lower speeds (20-40%)** for precise movements
- Use **higher speeds (50-70%)** for long straight runs
- **PID ramp-down** is implemented in `move_tank_for_cm()` for precision stopping
- **Acceleration ramping** is implemented in `turn_PID()` to prevent wheel slip

## Notes for AI Agents

When modifying or extending this code:

1. Maintain the existing function signatures
2. Keep the async/await pattern consistent
3. Add new mission functions following the established template
4. Document physical constants if hardware changes
5. Test incrementally - don't write entire mission sequences without validation
6. Consider edge cases: What if the robot overshoots? What if alignment is off?
7. Use type hints for new functions (e.g., `-> None`, `: int`, `: float`)

---

**Last Updated**: January 2026
