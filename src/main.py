from hub import port, motion_sensor
import motor, motor_pair, time
import runloop

# Constants - קבועים גלובליים בשימוש
WHEEL_CIRCUMFERENCE = 17.5929  # cm - For 56mm diameter wheels (small wheels)
WHEEL_BASE = 11.5  # מרחק בין הגלגלים cm

# PID Constants for turning (tune these values for your robot)
TURN_KP = 2.0
TURN_KI = 0.0
TURN_KD = 0.5

# PID Constants for straight driving
DRIVE_KP = 0.8
DRIVE_KI = 0.003
DRIVE_KD = 0.0

# Pair the motors connected to ports D (Left) and C (Right) as PAIR_1
motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)


def cm_to_degrees(distance_cm: float) -> float:
    """
    Convert distance in centimeters to motor degrees.

    :param distance_cm: Distance in centimeters.
    :return: Number of motor degrees needed to travel that distance.
    """
    return (abs(distance_cm) / WHEEL_CIRCUMFERENCE) * 360


async def turn_PID(turn_degrees: int, speed: int = 100) -> None:
    """
    Precise point turn using PID control with gyro feedback.

    Features: Shortest path logic, acceleration ramping, and safety timeout.

    :param turn_degrees: The degrees to turn (e.g., 90, -45, 180). Positive = clockwise.
    :param speed: Maximum speed (ramping controls actual speed).
    """

    # המתנה 0.05 שניות
    await runloop.sleep_ms(100)

    # איפוס ה-Yaw
    motion_sensor.reset_yaw(0)

    # Use module-level PID constants for easier tuning
    kp = TURN_KP
    ki = TURN_KI
    kd = TURN_KD
    # משתני עזר לבקרה
    integral = 0
    last_error = 0

    # מהירות מקסימלית ומינימלית (כדי שהרובוט לא ייתקע או ישתולל)
    max_speed = 100
    min_speed = 5
    Correction = 0
    ramp_up_step = 4  # Max speed increase per 10ms (Prevents wheel slip)
    current_out_speed = 0  # Track speed for ramping
    settle_count = 0

    # משתני יציבות - מוודא שהרובוט עצר בתוך היעד למשך זמן מסוים
    # settle_time = 0
    start_time = time.ticks_ms()
    timeout_sec = 5

    # Control loop - לולאת הבקרה
    while True:
        # Check Timeout (Safety)
        if (time.ticks_ms() - start_time) > (timeout_sec * 1000):
            print("Turn timed out!")
            break

        # Give the sensor a moment to reset
        time.sleep_ms(10)

        # קריאת הזווית הנוכחית (המרה מ-decidegrees לדרגות)
        current_yaw = motion_sensor.tilt_angles()[0] / 10

        # חישוב השגיאה (המרחק מהיעד)
        # error = turn_degrees - current_yaw
        error = (turn_degrees - current_yaw + 180) % 360 - 180

        print(
            "Left Speed:",
            speed,
            "Right Speed:",
            -speed,
            "Error:",
            error,
            "Correction:",
            Correction,
            "Yaw:",
            current_yaw,
        )

        # חישוב רכיבי ה-PID
        proportional = error * kp

        # Integral Windup protection: אל תגדיל את האינטגרל אם המנועים כבר במקסימום
        if abs(proportional) < max_speed:
            integral += error
        i_term = integral * ki

        # חישוב שגיאה דיפרנציאלית (הפרש)
        derivative = (error - last_error) * kd
        last_error = error

        # חישוב עוצמת התיקון הסופית
        Correction = proportional + i_term + derivative

        # Acceleration Ramping (Slew Rate)
        # Limits how fast the speed can increase to prevent jerking
        if abs(Correction) > abs(current_out_speed) + ramp_up_step:
            if Correction > 0:
                current_out_speed += ramp_up_step
            else:
                current_out_speed -= ramp_up_step
        else:
            current_out_speed = Correction

        # הגבלת מהירות (Clamping)
        # speed = max(min(Correction, max_speed), -max_speed)
        speed = max(min(current_out_speed, max_speed), -max_speed)

        # טיפול ב"אזור מת" - אם המהירות נמוכה מדי, המנוע לא יזוז
        if 0 < speed < min_speed:
            speed = min_speed
        if -min_speed < speed < 0:
            speed = -min_speed

        # המתנה קצרה למחזור הבקרה (חשוב לחישוב ה-Derivative)
        await runloop.sleep_ms(10)

        # turn at default velocity
        motor_pair.move_tank(motor_pair.PAIR_1, int(-speed), int(speed))

        # Exit Conditions
        if abs(error) < 0.5:
            settle_count += 1
            if settle_count > 12:  # Must be stable for 120ms
                break
        else:
            settle_count = 0

        await runloop.sleep_ms(10)  # קצב דגימה קבוע של 100Hz

    motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)  # עצירת התנועה
    await runloop.sleep_ms(10)  # קצב דגימה קבוע של 100Hz


async def move_tank_for_cm(move_cm: float, speed_per: int = 50) -> None:
    """
    Drive the robot a specified distance using gyro-based steering correction.

    :param move_cm: Distance in cm. Positive = forward, negative = backward.
    :param speed_per: Speed as percentage (0-100%).
    """
    # המתנה 0.05 שניות
    await runloop.sleep_ms(50)

    # איפוס ה-Yaw לנסיעה ישרה
    motion_sensor.reset_yaw(0)

    # Use module-level PID constants for easier tuning
    kp = DRIVE_KP
    ki = DRIVE_KI
    kd = DRIVE_KD

    # Convert speed percentage to hub velocity
    base_speed = int(speed_per * 10)

    # Determine direction: 1 for forward, -1 for backward
    direction = -1 if move_cm >= 0 else 1

    # Convert cm to degrees for distance tracking
    degrees_to_move = cm_to_degrees(move_cm)

    # Reset motor encoders to track distance traveled
    motor.reset_relative_position(port.D, 0)
    motor.reset_relative_position(port.C, 0)

    # אתחול פרמטרים
    last_error = 0
    integral = 0

    # Control loop - לולאת הבקרה
    while True:
        # Give the sensor a moment
        # time.sleep_ms(10)

        # Check distance traveled (average of both wheels)
        left_pos = abs(motor.relative_position(port.D))
        right_pos = abs(motor.relative_position(port.C))
        avg_pos = (left_pos + right_pos) / 2

        # Stop if we've traveled far enough
        remaining_dist = degrees_to_move - avg_pos
        if remaining_dist <= 0:
            break

        # Dynamic Speed (Ramp-down)
        # Slow down as we reach the target for 2025 precision standards
        current_base_speed = base_speed
        if remaining_dist < 300:  # If less than 300 degrees left
            current_base_speed = max(100, base_speed * (remaining_dist / 300))

        # PID Steering Logic
        # Get current yaw - we want to maintain 0 degrees (straight)
        # Yaw measurement: 1 degree = 10 decidegrees
        current_yaw = motion_sensor.tilt_angles()[0] / 10

        # Target yaw is 0 (straight)
        error = 0 - current_yaw  # We want yaw to be 0

        p_term = error * kp
        integral += error
        i_term = integral * ki
        d_term = (error - last_error) * kd

        steering_correction = int(p_term + i_term + d_term)
        last_error = error

        # Apply Power
        # Forward: Left = Base + Correction, Right = Base - Correction
        vel_l = (current_base_speed * direction) + steering_correction
        vel_r = (current_base_speed * direction) - steering_correction

        # Debug output
        print(
            "Left Vel:",
            vel_l,
            "Right Vel:",
            vel_r,
            "Yaw:",
            current_yaw,
            "Distance:",
            int(avg_pos),
            "/",
            int(degrees_to_move),
        )

        # Drive the motors
        motor_pair.move_tank(motor_pair.PAIR_1, int(vel_l), int(vel_r))
        await runloop.sleep_ms(10)

    # Stop with brake
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
    await runloop.sleep_ms(50)


# פונקציה להזזת המנוע הימני A
async def turn_right_arm(degrees_turn: int, speed_per: int):
    # Move Side Motor A backward (-270 degrees)
    fast = int(speed_per * 10)
    await motor.run_for_degrees(port.A, degrees_turn, fast)


# פונקציה להזזת המנוע הימני B
async def turn_left_arm(degrees_turn: int, speed_per: int):
    # Move Side Motor A backward (-270 degrees)
    fast = int(speed_per * 10)
    await motor.run_for_degrees(port.B, degrees_turn, fast)


# This is how to write the missions code. After writing the code, run it at the main function.
async def mission_one_and_two():
    print("--- Starting Mission 1+2 ---")
    # Move forward 10 cm
    # await move_tank_for_cm(10, 20)
    # סיבוב ימינה 90
    print("Turning Left")
    await turn_PID(-90, 40)

    print("Turning Right")
    await turn_PID(90, 40)

    # arm function here...
    await turn_right_arm(90, 50)
    await turn_right_arm(-90, 50)
    await turn_left_arm(-270, 50)


async def mission_three_and_four():
    print("--- Starting Mission 3+4 ---")
    await move_tank_for_cm(-8, 25)
    await turn_left_arm(-90, 50)
    await move_tank_for_cm(-3, 25)
    await turn_left_arm(200, 50)
    await move_tank_for_cm(11.5, 20)


async def oscillate_arm(
    arm_func, motor_degrees: int, speed: int, count: int = 4
) -> None:
    """
    Oscillate an arm motor back and forth a specified number of times.

    :param arm_func: The arm function to call (turn_right_arm or turn_left_arm).
    :param motor_degrees: The degrees to move in each direction.
    :param speed: The speed percentage (0-100%).
    :param count: Number of oscillation cycles.
    """
    for _ in range(count):
        await arm_func(-motor_degrees, speed)
        await arm_func(motor_degrees, speed)


async def mission_eight(move_speed: int = 40, arm_speed: int = 50) -> None:
    """Mission 8: Move forward, oscillate right arm, return."""
    print("--- Starting Mission 8 ---")

    # Move forward 37 cm
    await move_tank_for_cm(-37, move_speed)

    # Oscillate right arm 10 times
    await oscillate_arm(turn_right_arm, 70, arm_speed)

    # Move backward 40 cm
    await move_tank_for_cm(40, move_speed)


async def mission_ten(move_speed=40, turning_speed=40):
    print("--- Starting Mission 10 ---")

    # Move forward 15 cm
    await move_tank_for_cm(-2, move_speed)

    print("Turning Left")
    await turn_PID(90, turning_speed)

    # Move forward 15 cm
    await move_tank_for_cm(-35, move_speed)

    print("Turning right")
    await turn_PID(-90, turning_speed)
    # מפיל את הכד ותופס את התיבה
    await move_tank_for_cm(-3, move_speed)
    await move_tank_for_cm(3, move_speed)

    # להסתובב אחורה חזרה
    print("Turning right")
    await turn_PID(-90, turning_speed)
    await move_tank_for_cm(70, move_speed)


async def main():
    # הפעל את משימה 1+2
    await mission_one_and_two()

    # הפעל את משימה 3+4
    await mission_three_and_four()

    # הפעל את משימה 8
    await mission_eight()

    # הפעל את משימה 10
    await mission_ten()


runloop.run(main())
