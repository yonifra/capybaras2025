## Here the code for the Robot will be implemented
print("Robot code goes here")
print("hello")
from hub import light_matrix, port
import runloop, motor_pair, sys, math

# Pair the motors connected to ports D (Left) and C (Right) as PAIR_1
motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)

# Function to convert distance in cm to degrees for the motor encoders
def cm_to_degrees(distance_cm):
    # Calculate the number of degrees needed for the given distance
    # (distance / circumference) * 360 degrees per rotation
    WHEEL_CIRCUMFERENCE = 27.6 # Verify with current wheel
    return (abs(distance_cm) / WHEEL_CIRCUMFERENCE) * 360

async def move_tank_for_cm (move_cm, speed_per):
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
    degrees_to_move=int(degrees_to_move)
    #Print the calculated movement in beeing prefoemed
    #print(f"Moving {move_cm} cm at {speed_per}% speed ({degrees_to_move} degrees at {hub_velocity} velocity)")
    print(hub_velocity)
    # Move the robot straight in the degrees and velocity required 
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1, degrees_to_move, hub_velocity, hub_velocity)
    #Stops the motors
    await motor_pair.stop



#This is how to write the missions code. After writing the code, run it at the main function.
async def mission_one_and_two():
    print("--- Starting Mission 1+2 ---")
    # Move forward 70 cm
    await move_tank_for_cm (50, 50)
    # turn function here...
    # arm function here...


async def main():
    #run mission 1+2
    await mission_one_and_two()


runloop.run(main())
