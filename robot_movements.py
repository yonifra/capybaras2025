#!/usr/bin/env pybricks-micropython
"""
LEGO Robotics Challenge - Robot Movement Program
This program controls a robot through a series of movements for a robotics challenge.
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize sensors
color_sensor = ColorSensor(Port.S3)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

# Create a DriveBase for easier robot control
# Parameters: left_motor, right_motor, wheel_diameter (mm), axle_track (mm)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# Robot Challenge Mission: Navigate obstacle course
ev3.speaker.beep()
wait(500)

# Mission 1: Drive forward to first checkpoint
ev3.screen.print("Mission 1: Forward")
robot.straight(300)  # Move forward 300mm
wait(1000)

# Mission 2: Turn right 90 degrees
ev3.screen.print("Mission 2: Turn Right")
robot.turn(90)
wait(500)

# Mission 3: Drive forward to second checkpoint
ev3.screen.print("Mission 3: Forward")
robot.straight(200)  # Move forward 200mm
wait(1000)

# Mission 4: Check for obstacle with ultrasonic sensor
ev3.screen.print("Mission 4: Check Obstacle")
distance = ultrasonic_sensor.distance()
if distance < 100:  # If obstacle within 100mm
    ev3.speaker.beep(frequency=1000, duration=200)
    robot.straight(-100)  # Back up 100mm
    robot.turn(-90)  # Turn left to avoid
    robot.straight(150)  # Drive around obstacle
    robot.turn(90)  # Turn back to original direction
else:
    robot.straight(100)  # Continue forward
wait(1000)

# Mission 5: Follow line to finish zone
ev3.screen.print("Mission 5: Line Follow")
for i in range(10):  # Follow line for 10 iterations
    detected_color = color_sensor.color()
    if detected_color == Color.BLACK:
        # On the line, go straight
        robot.drive(100, 0)
    else:
        # Off the line, turn to find it
        robot.drive(50, 30)
    wait(100)

robot.stop()

# Mission 6: Turn left 90 degrees
ev3.screen.print("Mission 6: Turn Left")
robot.turn(-90)
wait(500)

# Mission 7: Final approach to target
ev3.screen.print("Mission 7: Final Approach")
robot.straight(250)  # Move forward 250mm
wait(1000)

# Mission Complete!
robot.stop()
ev3.screen.print("Mission Complete!")
ev3.speaker.beep(frequency=500, duration=200)
wait(200)
ev3.speaker.beep(frequency=700, duration=200)
wait(200)
ev3.speaker.beep(frequency=900, duration=400)
