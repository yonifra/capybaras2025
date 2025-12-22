from hub import port, motion_sensor
import motor, motor_pair, time
import runloop, sys, math

# Constants - קבועים גלובליים בשימוש
WHEEL_CIRCUMFERENCE = 17.5929  # לגלגל גדול
WHEEL_BASE = 11.5  # מרחק בין הגלגלים cm
# Distance from wheel axle to robot's center of mass in cm (forward)
CENTER_OFFSET = -4.0

# Pair the motors connected to ports D (Left) and C (Right) as PAIR_1
motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)


# פונקציה שמחזירה כמות מעלות נדרשות בהתאם לכמות הס״מ שהגלגל נסע
def cm_to_degrees(distance_cm):
    # Calculate the number of degrees needed for the given distance
    # (distance / circumference) * 360 degrees per rotation
    return (abs(distance_cm) / WHEEL_CIRCUMFERENCE) * 360


# פונקציה לסיבוב הרובוט עם ג׳יירו
async def turn_PID(turn_degrees: int, speed=100):
    """
    Advanced PID with Shortest Path Logic, Acceleration Ramping, and Safety Timeout.
    Executes a precise point turn using a Proportional-Integral (PI) controller with yaw feedback.
    :param turn_degrees: The degrees needed to turn (e.g., 90, -45, 180 degrees).
    :param kp: The proportional constant (tune this value, start around 1.0 to 3.0).
    :param ki: The integral constant (tune this value, start around 0.01 to 0.1).
    :param Kd: The Diffentail constant (Tune this value, Start around 0.2 to 0.9)
    """

    # המתנה 0.05 שניות
    await runloop.sleep_ms(100)

    # איפוס ה-Yaw
    motion_sensor.reset_yaw(0)

    # מקדמי בקרה (דורש כיול בהתאם לרובוט שלך)
    kp = 2  # Proportional: עוצמת התיקון הבסיסית
    ki = 0.0  # Integral: תיקון הצטברות שגיאה (עוזר להגיע בדיוק לזווית)
    kd = 0.5  # Derivative: "בלם" למניעת חריגה מהיעד
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

    # Conrtol loop - לולאת הבקרה
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

        # עדכון השגיאה האחרונה למחזור הבא
        last_error = error
        await runloop.sleep_ms(10)  # קצב דגימה קבוע של 100Hz

    motor_pair.stop(motor_pair.PAIR_1)  # עצירת התנועה
    await runloop.sleep_ms(10)  # קצב דגימה קבוע של 100Hz


async def move_tank_for_cm(move_cm, speed_per=50):
    """
    Drives the robot a specified distance in centimeters using gyro-based correction.
    Uses a similar approach to turn_PID() for stable, wobble-free straight driving.
    :param move_cm: The distance to travel in centimeters. Positive for forward, negative for backward.
    :param speed_per: The speed as a percentage (0 to 100%).
    """
    # המתנה 0.05 שניות
    await runloop.sleep_ms(50)

    # איפוס ה-Yaw לנסיעה ישרה
    motion_sensor.reset_yaw(0)

    # PID Constants for steering (Gyro)
    kp = 0.8  # How aggressively it returns to straight
    ki = 0.003  # Fixes slow drifts
    kd = 0.0  # Prevents "fishtailing" (wobbling)

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

        current_yaw = motion_sensor.tilt_angles()[0] / 10

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
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    # motor_pair.stop(motor_pair.PAIR_1)
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


async def mission_eight(move_speed=40, turning_speed=50):
    print("--- Starting Mission 8 ---")

    await move_tank_for_cm(-40, move_speed)

    # print("הרמת זרוע ימין")
    await turn_right_arm(90, turning_speed)
    await turn_right_arm(-90, turning_speed)
    await turn_right_arm(90, turning_speed)
    await turn_right_arm(-90, turning_speed)
    await turn_right_arm(90, turning_speed)
    await turn_right_arm(-90, turning_speed)
    await turn_right_arm(90, turning_speed)
    await turn_right_arm(-90, turning_speed)
    await turn_right_arm(90, turning_speed)
    await turn_right_arm(-90, turning_speed)

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
