## קוד לתחרות

from hub import light_matrix, port, motion_sensor, button
import motor, motor_pair, time
import runloop, sys, math

# Constants - קבועים גלובליים בשימוש
WHEEL_CIRCUMFERENCE = 27.6# לגלגל גדול
WHEEL_BASE = 11.5# מרחק בין הגלגלים cm
# Distance from wheel axle to robot's center of mass in cm (forward)
CENTER_OFFSET = -4.0

# Pair the motors connected to ports D (Left) and C (Right) as PAIR_1
motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)


# פונקציה שמחזירה כמות מעלות נדרשות בהתאם לכמות הס״מ שהגלגל נסע
def cm_to_degrees(distance_cm):
    # Calculate the number of degrees needed for the given distance
    # (distance / circumference) * 360 degrees per rotation
    return (abs(distance_cm) / WHEEL_CIRCUMFERENCE) * 360

#פונקציה לסיבוב הרובוט עם ג׳יירו
async def turn_pi(turn_degrees: int, speed: int):
    """
    Executes a precise point turn using a Proportional-Integral (PI) controller with yaw feedback.
    :param turn_degrees: The degrees needed to turn (e.g., 90, -45, 180 degrees).
    :param speed: The base speed limit for turning.
    :param kp: The proportional constant (tune this value, start around 1.0 to 3.0).
    :param ki: The integral constant (tune this value, start around 0.01 to 0.1).
    """

    #המתנה 0.05 שניות
    await runloop.sleep_ms(50)

    # איפוס ה-Yaw
    motion_sensor.reset_yaw(0)

    #מקדמי בקרה
    kp: float = 2.5
    ki: float = 0.05
    # קביעת מהירות הסיבוב ההתחלתית
    speed = 40
    Vel_R=speed
    Vel_L=speed

    #חישוב השגיאה
    error_PID = abs(turn_degrees - motion_sensor.tilt_angles()[0]/10)

    #Conrtol loop - לולאת הבקרה
    while error_PID>1:
        # Give the sensor a moment to reset
        time.sleep_ms(10)
        #Calculate the correction needed
        error_PID= abs(turn_degrees - motion_sensor.tilt_angles()[0]/10)

        #חשוב התיקון הנדרש * המקדם בקרה
        Correction = int(error_PID*kp)

        #בדיקת כיוון הסיבוב - חיובי=שמאלה, שלילי=ימינה
        if turn_degrees > 0:
            Vel_R = Correction
            Vel_L = -Correction
        elif turn_degrees < 0:
            Vel_R = -Correction
            Vel_L = Correction

        #Yaw measurment: 1 degree = 10 decidegrees
        #Yaw, Pitch, Roll = motion_sensor.tilt_angles()
        Yaw = motion_sensor.tilt_angles()[0]/10
        print('Left Vel:', Vel_L, 'Right Vel:', Vel_R, 'Error:', error_PID, 'Correction:', Correction, 'Yaw:',Yaw)

        # turn at default velocity
        motor_pair.move_tank(motor_pair.PAIR_1,Vel_L,Vel_R)

    motor_pair.stop(motor_pair.PAIR_1)


async def move_tank_for_cm(move_cm, speed_per=10):
    """Drives the robot a specified distance in centimeters at a given % speed (0-100%).
    Uses gyro sensor with PI control to maintain a straight heading.
    parameter move_cm: The distance to travel in centimeters. Positive for forward, negative for backward.
    parameter speed_per: The speed as a percentage (0 to 100%).
    """
    # Wait briefly before starting
    await runloop.sleep_ms(50)

    # Reset the gyro sensor to 0 for straight-line driving
    motion_sensor.reset_yaw(0)

    # PI control constants for heading correction
    kp = 2.0# Proportional constant - adjusts how aggressively to correct
    ki = 0.01# Integral constant - helps eliminate steady-state error

    # Convert input speed (percentage 0-100%) to Hub velocity (degrees/second, usually max 1000)
    hub_velocity = int(speed_per * 10)

    # Convert cm to degrees
    degrees_to_move = cm_to_degrees(move_cm)

    # Determine direction
    direction = 1 if move_cm >= 0 else -1

    # Reset motor encoders to track distance traveled
    motor.reset_relative_position(port.D, 0)
    motor.reset_relative_position(port.C, 0)

    # Integral term accumulator
    integral = 0.0

    # Control loop - continue until we've traveled the required distance
    while True:
        # Get current encoder positions (average of both wheels)
        left_pos = abs(motor.relative_position(port.D))
        right_pos = abs(motor.relative_position(port.C))
        avg_pos = (left_pos + right_pos) / 2

        # Check if we've traveled far enough
        if avg_pos >= degrees_to_move:
            break

        # Get current heading from gyro (in degrees, yaw is in decidegrees)
        current_yaw = motion_sensor.tilt_angles()[0] / 10

        # Calculate heading error (we want to maintain 0 degrees)
        error = current_yaw

        # Update integral term
        integral += error

        # Calculate correction using PI controller
        correction = int(kp * error + ki * integral)

        # Apply correction to motor velocities
        # If robot drifts right (positive yaw), slow down right wheel / speed up left
        # If robot drifts left (negative yaw), slow down left wheel / speed up right
        base_vel = hub_velocity * direction
        left_vel = base_vel + correction
        right_vel = base_vel - correction
       # left_vel = (hub_velocity - correction) * direction
       # right_vel = (hub_velocity + correction) * direction

        # Clamp velocities to valid range
        left_vel = max(-1000, min(1000, left_vel))
        right_vel = max(-1000, min(1000, right_vel))

        # Drive the motors
        motor_pair.move_tank(motor_pair.PAIR_1, int(left_vel), int(right_vel))

        # Small delay for control loop
        await runloop.sleep_ms(10)

    # Stop the motors with brake
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)



#פונקציה להזזת המנוע הימני A
async def turn_right_arm(degrees_turn: int, speed_per: int):
    # Move Side Motor A backward (-270 degrees)
    fast=int(speed_per * 10)
    await motor.run_for_degrees(port.A, degrees_turn, fast)


#פונקציה להזזת המנוע הימני B
async def turn_left_arm(degrees_turn: int, speed_per: int):
    # Move Side Motor A backward (-270 degrees)
    fast=int(speed_per * 10)
    await motor.run_for_degrees(port.B, degrees_turn, fast)




# This is how to write the missions code. After writing the code, run it at the main function.
async def mission_one_and_two():
    move_speed = 40
    print("--- Starting Mission 1+2 ---")
    # Move forward 10 cm
    await move_tank_for_cm(-85, move_speed)
    print('Turning Left')
    await turn_pi(45, 40)
    await move_tank_for_cm(-5, move_speed)
    print("הרמת זרוע ימין")
    await turn_right_arm(90,50)
    await move_tank_for_cm(5, move_speed)
    await turn_pi(-45, 40)
    await move_tank_for_cm(42, move_speed)
    await turn_pi(90, 40)
    await turn_left_arm(90,50)
    await move_tank_for_cm(7, move_speed)
    await turn_pi(-90, 40)
    await move_tank_for_cm(50, move_speed)

async def main():
    # הפעל את משימה 1+2
    await mission_one_and_two()

    # הפעל את משימה 3+4


runloop.run(main())
