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
    parameter move_cm: The distance to travel in centimeters. Positive for forward, negative for backward.
    parameter speed_per: The speed as a percentage (0 to 100%).
    """
    # Convert input speed (percentage 0-100%) to Hub velocity (degrees/second, usually max 1000)
    # 50% is roughly 500 deg/s
    hub_velocity = int(speed_per * 10)

    # Convert cm to degrees
    degrees_to_move = cm_to_degrees(move_cm)

    # If distance is negative, the hub_velocity should also be negative to reverse direction in the move tank for degrees function
    if move_cm < 0:
        hub_velocity = -abs(hub_velocity)# Ensure velocity matches direction sign
    hub_velocity = int(hub_velocity)
    degrees_to_move = int(degrees_to_move)
    # Print the calculated movement being performed
    # print(f"Moving {move_cm} cm at {speed_per}% speed ({degrees_to_move} degrees at {hub_velocity} velocity)")
    print(hub_velocity)

    # Move the robot straight in the degrees and velocity required
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, degrees_to_move, hub_velocity, hub_velocity)

    # Stops the motors
    motor_pair.stop(motor_pair.PAIR_1)


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
    print("--- Starting Mission 1+2 ---")
    # Move forward 10 cm
    #await move_tank_for_cm(10, 20)
    #סיבוב ימינה 90
    print('Turning Left')
    await turn_pi(-90, 40)

    print('Turning Right')
    await turn_pi(90, 40)

    # arm function here...
    await turn_right_arm(90,50)
    await turn_right_arm(-90,50)
    await turn_left_arm(-270,50)


async def main():
    # הפעל את משימה 1+2
    await mission_one_and_two()

    # הפעל את משימה 3+4


runloop.run(main())
