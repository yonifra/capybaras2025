from hub import port, motion_sensor
import motor, motor_pair, time
import runloop, sys, math

# Constants - קבועים גלובליים בשימוש
WHEEL_CIRCUMFERENCE = 17.5  # לגלגל קטן
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
async def turn_pi(turn_degrees: int, speed: int):
    """
    Executes a precise point turn using a Proportional-Integral (PI) controller with yaw feedback.
    :param turn_degrees: The degrees needed to turn (e.g., 90, -45, 180 degrees).
    :param speed: The base speed limit for turning.
    :param kp: The proportional constant (tune this value, start around 1.0 to 3.0).
    :param ki: The integral constant (tune this value, start around 0.01 to 0.1).
    """

    # המתנה 0.05 שניות
    await runloop.sleep_ms(50)

    # איפוס ה-Yaw
    motion_sensor.reset_yaw(0)

    # מקדמי בקרה
    kp: float = 2.5
    ki: float = 0.05
    # קביעת מהירות הסיבוב ההתחלתית
    speed = 40
    Vel_R = speed
    Vel_L = speed

    # חישוב השגיאה
    error_PID = abs(turn_degrees - motion_sensor.tilt_angles()[0] / 10)

    # Conrtol loop - לולאת הבקרה
    while error_PID > 1:
        # Give the sensor a moment to reset
        time.sleep_ms(10)
        # Calculate the correction needed
        error_PID = abs(turn_degrees - motion_sensor.tilt_angles()[0] / 10)

        # חשוב התיקון הנדרש * המקדם בקרה
        Correction = int(error_PID * kp)

        # בדיקת כיוון הסיבוב - חיובי=שמאלה, שלילי=ימינה
        if turn_degrees > 0:
            Vel_R = Correction
            Vel_L = -Correction
        elif turn_degrees < 0:
            Vel_R = -Correction
            Vel_L = Correction

        # Yaw measurment: 1 degree = 10 decidegrees
        # Yaw, Pitch, Roll = motion_sensor.tilt_angles()
        Yaw = motion_sensor.tilt_angles()[0] / 10
        print(
            "Left Vel:",
            Vel_L,
            "Right Vel:",
            Vel_R,
            "Error:",
            error_PID,
            "Correction:",
            Correction,
            "Yaw:",
            Yaw,
        )

        # turn at default velocity
        motor_pair.move_tank(motor_pair.PAIR_1, Vel_L, Vel_R)

    motor_pair.stop(motor_pair.PAIR_1)


async def move_tank_for_cm(move_cm, speed_per=50):
    """
    Drives the robot a specified distance in centimeters using gyro-based correction.
    Uses a similar approach to turn_pi() for stable, wobble-free straight driving.
    :param move_cm: The distance to travel in centimeters. Positive for forward, negative for backward.
    :param speed_per: The speed as a percentage (0 to 100%).
    """
    # המתנה 0.05 שניות
    await runloop.sleep_ms(50)

    # איפוס ה-Yaw לנסיעה ישרה
    motion_sensor.reset_yaw(0)

    # מקדם בקרה - lower value = smoother, less wobble
    kp: float = 1.5

    # Convert speed percentage to hub velocity
    base_speed = int(speed_per * 10)

    # Determine direction: 1 for forward, -1 for backward
    direction = 1 if move_cm >= 0 else -1

    # Convert cm to degrees for distance tracking
    degrees_to_move = cm_to_degrees(move_cm)

    # Reset motor encoders to track distance traveled
    motor.reset_relative_position(port.D, 0)
    motor.reset_relative_position(port.C, 0)

    # Control loop - לולאת הבקרה
    while True:
        # Give the sensor a moment
        time.sleep_ms(10)

        # Check distance traveled (average of both wheels)
        left_pos = abs(motor.relative_position(port.D))
        right_pos = abs(motor.relative_position(port.C))
        avg_pos = (left_pos + right_pos) / 2

        # Stop if we've traveled far enough
        if avg_pos >= degrees_to_move:
            break

        # Get current yaw - we want to maintain 0 degrees (straight)
        # Yaw measurement: 1 degree = 10 decidegrees
        current_yaw = motion_sensor.tilt_angles()[0] / 10

        # Calculate correction - how much we've drifted from straight (0 degrees)
        # חישוב התיקון הנדרש * מקדם הבקרה
        correction = int(current_yaw * kp)

        # Apply base speed with direction
        Vel_L = base_speed * direction
        Vel_R = base_speed * direction

        # Apply correction to maintain heading of 0 degrees
        # If yaw is positive (drifted right), slow right wheel / speed up left
        # If yaw is negative (drifted left), slow left wheel / speed up right
        Vel_L = Vel_L + correction
        Vel_R = Vel_R - correction

        # Debug output
        print(
            "Left Vel:",
            Vel_L,
            "Right Vel:",
            Vel_R,
            "Yaw:",
            current_yaw,
            "Distance:",
            int(avg_pos),
            "/",
            int(degrees_to_move),
        )

        # Drive the motors
        motor_pair.move_tank(motor_pair.PAIR_1, Vel_L, Vel_R)

    # Stop with brake
    motor_pair.stop(motor_pair.PAIR_1)


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
    await turn_pi(-90, 40)

    print("Turning Right")
    await turn_pi(90, 40)

    # arm function here...
    await turn_right_arm(90, 50)
    await turn_right_arm(-90, 50)
    await turn_left_arm(-270, 50)


async def mission_three_and_four():  # missions 3+4
    print("--- Starting Mission 3+4 ---")
    # Move forward 10 cm
    # await move_tank_for_cm(-80, 55)
    # סיבוב ימינה 90
    # print('Turning Left')
    # await turn_pi(-90, 40)

    # print('Turning Right')

    # arm function here...
    # await turn_right_arm(90,50)
    # await turn_right_arm(-90,50)
    # await turn_left_arm(-270,50)
    # await turn_pi(-90,40)

    # await move_tank_for_cm(-15, 50)

    # await turn_pi(-45, 40)

    # await move_tank_for_cm(34, 30)

    # await turn_pi(45, 40)

    await move_tank_for_cm(-8, 25)
    await turn_left_arm(-90, 50)
    await move_tank_for_cm(-3, 25)
    await turn_left_arm(200, 50)

    # await turn_pi(-2,30)

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
    await turn_pi(90, turning_speed)

    # Move forward 15 cm
    await move_tank_for_cm(-35, move_speed)

    print("Turning right")
    await turn_pi(-90, turning_speed)
    # מפיל את הכד ותופס את התיבה
    await move_tank_for_cm(-3, move_speed)
    await move_tank_for_cm(3, move_speed)

    # להסתובב אחורה חזרה
    print("Turning right")
    await turn_pi(-90, turning_speed)
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
