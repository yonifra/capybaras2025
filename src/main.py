## Here the code for the Robot will be implemented

from hub import light_matrix, port, motion_sensor
import motor
import runloop, motor_pair, sys, math

# Constants
WHEEL_CIRCUMFERENCE = 27.6  # Verify with current wheel
WHEEL_BASE = 11.5  # Distance between left and right wheels in cm
CENTER_OFFSET = -4.0  # Distance from wheel axle to robot's center of mass in cm (forward)

# Pair the motors connected to ports D (Left) and C (Right) as PAIR_1
motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)

# Function to convert distance in cm to degrees for the motor encoders
def cm_to_degrees(distance_cm):
    # Calculate the number of degrees needed for the given distance
    # (distance / circumference) * 360 degrees per rotation
    return (abs(distance_cm) / WHEEL_CIRCUMFERENCE) * 360

async def rotate(degrees, speed_per=30):
    '''Rotates the robot by a specified angle using the gyro sensor with easing.
       parameter degrees: The angle to rotate in degrees. Positive for clockwise, negative for counter-clockwise.
       parameter speed_per: The speed as a percentage (0 to 100%). Default is 30%.
    '''
    # Print initial yaw value before reset
    initial_yaw = motion_sensor.tilt_angles()[0] / 10
    print("[ROTATE START] Initial yaw:", initial_yaw, "degrees, Target:", degrees, "degrees")

    # Reset the gyro sensor
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(50)  # Allow sensor to stabilize after reset

    # Convert speed percentage to motor velocity
    max_motor_speed = int(speed_per * 10)
    min_motor_speed = int(max_motor_speed * 0.2)  # Start at 20% of max speed

    # Determine rotation direction
    clockwise = degrees > 0
    target_angle = abs(degrees)

    # For in-place rotation, both wheels must move at the same speed in opposite directions
    # This rotates the robot around the midpoint between the two wheels

    # Easing parameters (in degrees)
    ease_in_angle = min(15, target_angle * 0.2)   # Accelerate for first 15° or 20% of turn
    ease_out_angle = min(20, target_angle * 0.3)  # Decelerate for last 20° or 30% of turn

    while True:
        current_angle = abs(motion_sensor.tilt_angles()[0] / 10)
        remaining_angle = target_angle - current_angle

        # Check if target reached - stop motors before breaking
        if remaining_angle <= 1:
            motor_pair.stop(motor_pair.PAIR_1, motor.BREAK)
            break

        # Calculate speed with easing
        if current_angle < ease_in_angle:
            # Ease in: gradually increase speed
            progress = current_angle / ease_in_angle
            base_motor_speed = int(min_motor_speed + (max_motor_speed - min_motor_speed) * progress)
        elif remaining_angle < ease_out_angle:
            # Ease out: gradually decrease speed
            progress = remaining_angle / ease_out_angle
            base_motor_speed = int(min_motor_speed + (max_motor_speed - min_motor_speed) * progress)
        else:
            # Full speed in the middle
            base_motor_speed = max_motor_speed

        # Apply equal speeds in opposite directions for in-place rotation
        if clockwise:
            # Clockwise: left forward, right backward
            left_speed = base_motor_speed
            right_speed = -base_motor_speed
        else:
            # Counter-clockwise: left backward, right forward
            left_speed = -base_motor_speed
            right_speed = base_motor_speed

        # Move with current speed
        motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
        await runloop.sleep_ms(10)

    # Motors already stopped in loop, just wait for robot to settle
    await runloop.sleep_ms(100)

    # Print final yaw value for debugging
    final_yaw = motion_sensor.tilt_angles()[0] / 10
    error = target_angle - abs(final_yaw)
    print("[ROTATE END] Final yaw:", final_yaw, "degrees, Target was:", degrees, "degrees, Error:", round(error, 2), "degrees")

async def move_tank_for_cm(move_cm, speed_per):
    '''Drives the robot a specified distance in centimeters at a given % speed (0-100%).
       parameter move_cm: The distance to travel in centimeters. Positive for forward, negative for backward.
       parameter speed_per: The speed as a percentage (0 to 100%).
    '''
    # Convert input speed (percentage 0-100%) to Hub velocity (degrees/second, usually max 1000)
    # 50% is roughly 500 deg/s
    hub_velocity = int(speed_per * 10)

    # Convert cm to degrees
    degrees_to_move = cm_to_degrees(move_cm)

    # If distance is negative, the hub_velocity should also be negative to reverse direction in the move tank for degrees function
    if move_cm < 0:
        hub_velocity = -abs(hub_velocity) # Ensure velocity matches direction sign
    hub_velocity = int(hub_velocity)
    degrees_to_move = int(degrees_to_move)
    # Print the calculated movement being performed
    # print(f"Moving {move_cm} cm at {speed_per}% speed ({degrees_to_move} degrees at {hub_velocity} velocity)")
    print(hub_velocity)
    # Move the robot straight in the degrees and velocity required
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, degrees_to_move, hub_velocity, hub_velocity)
    # Stops the motors
    motor_pair.stop(motor_pair.PAIR_1, motor.BREAK)

# This is how to write the missions code. After writing the code, run it at the main function.
async def mission_one_and_two():
    print("--- Starting Mission 1+2 ---")
    # Move forward 50 cm
    # await move_tank_for_cm(50, 50)
    # Example: Rotate 90 degrees clockwise
    await rotate(90, 30)
    # arm function here...


async def main():
    # Run mission 1+2
    await mission_one_and_two()


runloop.run(main())
