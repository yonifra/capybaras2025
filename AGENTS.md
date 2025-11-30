# AI Agent Instructions for Capybaras2025 Robot Project

## Project Overview

This is a **LEGO SPIKE Prime v3.0 robotics project** written in **Python (MicroPython)**. The robot uses motors, sensors, and the LEGO hub to complete missions. The project is structured for a robotics competition where the robot must navigate, manipulate objects, and complete specific tasks.

## Technology Stack

- **Language**: Python (MicroPython for LEGO SPIKE Prime)
- **Hardware**: LEGO SPIKE Prime Hub
- **Key Modules**: `hub`, `motor`, `motor_pair`, `runloop`, `motion_sensor`

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
    await rotate(90)

# Run with runloop
runloop.run(main())
```

### 2. **Motor Configuration**

- **Left Motor**: Port D
- **Right Motor**: Port C
- **Motor Pair**: `motor_pair.PAIR_1`
- Motors are paired at startup: `motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)`

### 3. **Physical Constants**

These constants are tuned for the specific robot and MUST be verified/updated if hardware changes:

```python
WHEEL_CIRCUMFERENCE = 27.6  # cm - Verify with current wheels
WHEEL_BASE = 11.5           # cm - Distance between left and right wheels
CENTER_OFFSET = -4.0        # cm - Distance from wheel axle to center of mass
```

### 4. **Core Movement Functions**

#### `move_tank_for_cm(move_cm, speed_per=10)`

- Moves robot forward/backward a specified distance
- `move_cm`: Distance in centimeters (positive = forward, negative = backward)
- `speed_per`: Speed as percentage (0-100%)
- Automatically converts cm to motor encoder degrees
- Always stops with `motor.BREAK` for precise stopping

#### `rotate(degrees, speed_percentage=30)`

- Rotates robot in place using gyro sensor
- `degrees`: Angle to rotate (positive = clockwise, negative = counter-clockwise)
- `speed_percentage`: Speed as percentage (0-100%), default 30%
- **Features**:
  - Resets gyro sensor before rotation
  - Easing (acceleration/deceleration) for smooth movement
  - In-place rotation (both wheels move opposite directions at equal speed)
  - Error reporting for debugging

### 5. **Mission Structure**

Organize code into mission functions:

```python
async def mission_one_and_two():
    print("--- Starting Mission 1+2 ---")
    await move_tank_for_cm(50, 50)  # Move forward 50cm at 50% speed
    await rotate(90)                 # Turn 90 degrees clockwise
    # Add mission-specific code here

async def main():
    await mission_one_and_two()
    # await mission_three()
    # etc.

runloop.run(main())
```

## Coding Guidelines for AI Agents

### DO ✅

1. **Always use async/await** for robot control functions
2. **Print debug messages** to track mission progress:
   ```python
   print("--- Starting Mission X ---")
   print(f"Moving {distance}cm at {speed}% speed")
   ```
3. **Stop motors explicitly** after movements: `motor_pair.stop(motor_pair.PAIR_1, motor.BREAK)`
4. **Add sleep delays** when needed: `await runloop.sleep_ms(100)`
5. **Use percentage-based speeds** (0-100%) for readability
6. **Comment complex sequences** to explain mission strategy
7. **Keep missions modular** - one function per mission or sub-task
8. **Test movements incrementally** - don't chain too many actions without validation points

### DON'T ❌

1. **Don't modify constants** without physical measurements
2. **Don't use blocking operations** - always use async functions
3. **Don't forget to pair motors** at initialization
4. **Don't mix up motor ports** (D=Left, C=Right)
5. **Don't skip gyro sensor reset** in rotation functions
6. **Don't use raw motor degrees** - use cm_to_degrees() helper
7. **Don't assume perfect accuracy** - account for slippage and calibration errors

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
motor_pair.stop(motor_pair.PAIR_1, motor.BREAK)
```

### Individual Motor Control (if needed)

```python
import motor
motor.run(port.A, velocity)
motor.run_for_degrees(port.A, degrees, velocity)
motor.stop(port.A, motor.BREAK)
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
4. **Implement movements** - Use `move_tank_for_cm()` and `rotate()`
5. **Add attachments/manipulations** - Control additional motors for arms/mechanisms
6. **Add to main()** - Call the mission function
7. **Test and tune** - Adjust speeds, distances, and angles based on performance

## Example Mission Template

```python
async def mission_example():
    """
    Mission Description:
    - Move to target area
    - Activate mechanism
    - Return to base
    """
    print("--- Starting Mission Example ---")

    # Phase 1: Navigate to target
    await move_tank_for_cm(60, 50)      # Forward 60cm at 50% speed
    await rotate(45)                     # Turn 45° clockwise
    await move_tank_for_cm(30, 30)      # Approach slowly

    # Phase 2: Execute task
    await runloop.sleep_ms(500)          # Brief pause
    # Add mechanism control here

    # Phase 3: Return
    await rotate(-45)                    # Turn back
    await move_tank_for_cm(-60, 50)     # Reverse to base

    print("--- Mission Example Complete ---")
```

## Debugging Tips

1. **Use print statements liberally** - They appear in the console
2. **Test movements individually** before chaining
3. **Check sensor values**: Print gyro readings to verify accuracy
4. **Adjust constants** if movements are consistently off
5. **Add sleep delays** between rapid movements for stability
6. **Watch for wheel slippage** on different surfaces

## Additional Ports (Future Use)

If using additional motors/sensors, common ports:

- Port A, B: Often used for attachments/mechanisms
- Port E, F: Additional motors or sensors
- Always document which port is used for what mechanism

## Performance Optimization

- Use **lower speeds (20-40%)** for precise movements
- Use **higher speeds (50-70%)** for long straight runs
- **Ease-in/ease-out** is already implemented in `rotate()` function
- Consider adding similar easing to `move_tank_for_cm()` if needed

## Notes for AI Agents

When modifying or extending this code:

1. Maintain the existing function signatures
2. Keep the async/await pattern consistent
3. Add new mission functions following the established template
4. Document physical constants if hardware changes
5. Test incrementally - don't write entire mission sequences without validation
6. Consider edge cases: What if the robot overshoots? What if alignment is off?

---

**Last Updated**: Generated for repository at commit c86206708bb8ec8a4586b7a2da20ff619f7dec6d
