# =============================================================================
# CAPYBARAS 2025 - LEGO SPIKE Prime Robot Control
# =============================================================================
# Main robot control code for FLL competition missions.
# Uses PID control for precise movements and gyro-based steering correction.
# =============================================================================

# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------
from hub import light_matrix, port, motion_sensor
import motor, motor_pair, time
import runloop

# -----------------------------------------------------------------------------
# PHYSICAL CONSTANTS (קבועים גלובליים)
# -----------------------------------------------------------------------------
# These values are calibrated for our specific robot hardware.
# DO NOT MODIFY without physical measurements!

WHEEL_CIRCUMFERENCE = 17.5929  # Wheel circumference in cm (for large wheels)
WHEEL_BASE = 11.5  # Distance between left and right wheels in cm
CENTER_OFFSET = -4.0  # Distance from wheel axle to robot's center of mass (forward)

# -----------------------------------------------------------------------------
# MOTOR INITIALIZATION
# -----------------------------------------------------------------------------
# Port Assignments:
#   - Port D: Left drive motor
#   - Port C: Right drive motor
#   - Port A: Right arm motor
#   - Port B: Left arm motor

motor_pair.pair(motor_pair.PAIR_1, port.D, port.C)


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================


def cm_to_degrees(distance_cm):
    """
    Convert a distance in centimeters to motor rotation degrees.

    פונקציה שמחזירה כמות מעלות נדרשות בהתאם לכמות הס״מ שהגלגל נסע

    :param distance_cm: Distance to travel in centimeters.
    :return: Number of motor degrees needed (always positive).

    Formula: (distance / wheel_circumference) * 360 degrees per rotation
    """
    return (abs(distance_cm) / WHEEL_CIRCUMFERENCE) * 360


# =============================================================================
# MOVEMENT CONTROL FUNCTIONS
# =============================================================================


async def turn_PID(turn_degrees: int):
    """
    Execute a precise point turn using PID control with gyro feedback.

    פונקציה לסיבוב הרובוט עם ג׳יירו (ימינה +) (שמאלה-)

    Features:
        - Shortest path logic (always takes optimal rotation direction)
        - Acceleration ramping to prevent wheel slip
        - Safety timeout (3 seconds max)
        - Settle detection for precise stopping
        - Integral windup protection

    :param turn_degrees: Target angle in degrees.
                         Positive = clockwise (right)
                         Negative = counter-clockwise (left)

    PID Constants (tuned for this robot):
        - kp: Proportional - base correction strength
        - ki: Integral - accumulated error correction
        - kd: Derivative - dampening to prevent overshoot
    """

    # -------------------------------------------------------------------------
    # ZIEGLER-NICHOLS PID TUNING (Optional)
    # -------------------------------------------------------------------------
    # If ku (critical gain) and tu (oscillation period) are provided,
    # calculate recommended PID values using Ziegler-Nichols method.
    #
    # How to calibrate using Ziegler-Nichols:
    # 1. Set ki=0 and kd=0
    # 2. Increase kp until robot oscillates with constant amplitude
    # 3. Record this kp value as Ku (critical gain)
    # 4. Measure time for 5 full oscillations, divide by 5 = Tu
    # 5. Run this function to get recommended PID values
    # -------------------------------------------------------------------------
    ku = 0  # Critical gain (found through manual calibration)
    tu = 0  # Oscillation period in seconds

    if ku > 0 and tu > 0:
        # Calculate Ziegler-Nichols recommended values
        recommended_kp = 0.6 * ku
        recommended_ki = 1.2 * ku / tu * 0.02  # Adjusted for 20ms sample period
        recommended_kd = 0.075 * ku * tu
        print("--- ZN Recommendations ---")
        print(
            "Kp: ", recommended_kp, ", Ki: ", recommended_ki, ", Kd: ", recommended_kd
        )
        kp, ki, kd = recommended_kp, recommended_ki, recommended_kd
    else:
        # -------------------------------------------------------------------------
        # DEFAULT PID CONSTANTS (Calibrated for this robot)
        # -------------------------------------------------------------------------
        kp = 3.6  # Proportional: Base correction strength
        ki = 0.0288  # Integral: Accumulated error correction (helps reach exact angle)
        kd = 2.25  # Derivative: Dampening to prevent overshoot

    # -------------------------------------------------------------------------
    # INITIALIZATION
    # -------------------------------------------------------------------------

    # Brief pause before starting
    await runloop.sleep_ms(100)

    # Reset gyro yaw to zero
    motion_sensor.reset_yaw(0)

    # PID control variables
    integral = 0
    last_error = 0

    # Speed limits (prevents motor stall or runaway)
    max_speed = 200  # Maximum motor velocity
    min_speed = 10  # Minimum motor velocity (prevents stalling)
    speed = 200
    Correction = 0

    # Acceleration ramping (prevents wheel slip on start)
    ramp_up_step = 20  # Max speed increase per 10ms cycle
    current_out_speed = 0  # Current ramped speed

    # Settle detection (ensures stable stop at target)
    settle_count = 0

    # Safety timeout
    start_time = time.ticks_ms()
    timeout_sec = 3  # Maximum turn duration

    # -------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # -------------------------------------------------------------------------
    while True:

        # --- Safety Timeout Check ---
        if (time.ticks_ms() - start_time) > (timeout_sec * 1000):
            print("Turn timed out!")
            break

        # Brief sensor reading delay
        time.sleep_ms(10)

        # --- Read Current Angle ---
        # Convert from decidegrees to degrees
        current_yaw = motion_sensor.tilt_angles()[0] / 10

        # --- Calculate Error (shortest path) ---
        # Uses modulo arithmetic to always take the shortest rotation path
        error = (turn_degrees - current_yaw + 180) % 360 - 180

        # --- PID Calculations ---

        # Proportional term: immediate response to error
        proportional = error * kp

        # Integral term with windup protection:
        # Don't accumulate if motors are already at max output
        if abs(proportional) < max_speed:
            integral += error
        i_term = integral * ki

        # Derivative term: rate of change (dampening)
        derivative = (error - last_error) * kd
        last_error = error

        # Combined PID output
        Correction = proportional + i_term + derivative

        # --- Acceleration Ramping (Slew Rate Limiting) ---
        # Prevents jerky starts by limiting speed change per cycle
        if abs(Correction) > abs(current_out_speed) + ramp_up_step:
            if Correction > 0:
                current_out_speed += ramp_up_step
            else:
                current_out_speed -= ramp_up_step
        else:
            current_out_speed = Correction

        # --- Speed Clamping ---
        # Limit speed to max/min bounds
        speed = max(min(current_out_speed, max_speed), -max_speed)

        # --- Dead Zone Handling ---
        # Ensure minimum speed to overcome motor friction
        if 0 < speed < min_speed:
            speed = min_speed
        if -min_speed < speed < 0:
            speed = -min_speed

        # Control loop delay (important for derivative calculation)
        await runloop.sleep_ms(10)

        # --- Apply Motor Power ---
        # Point turn: left and right motors spin in opposite directions
        motor_pair.move_tank(motor_pair.PAIR_1, int(-speed), int(speed))

        # --- Exit Condition ---
        # Stop when error is less than 1 degree
        if abs(error) < 1:
            settle_count += 1
            break
        else:
            settle_count = 0

        # Update last error for next iteration
        last_error = error
        await runloop.sleep_ms(10)  # Fixed 100Hz sample rate
    # --- Debug Output ---
    print(
        "Turning: ",
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

    # --- Stop and Hold Position ---
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    await runloop.sleep_ms(10)


# -----------------------------------------------------------------------------
# STRAIGHT LINE DRIVING
# -----------------------------------------------------------------------------
async def move_tank_for_cm(move_cm, speed_per=50):
    """
    Drives the robot a specified distance in centimeters using gyro-based correction.
    Uses a similar approach to turn_pi() for stable, wobble-free straight driving.
    :param move_cm: The distance to travel in centimeters. Positive for forward, negative for backward.
    :param speed_per: The speed as a percentage (0 to 100%).
    """
    # המתנה 0.05 שניות
    await runloop.sleep_ms(50)
    start_time = time.ticks_ms()
    # איפוס ה-Yaw לנסיעה ישרה
    motion_sensor.reset_yaw(0)

    # PID Constants for steering (Gyro)
    kp = 1.2  # How aggressively it returns to straight
    ki = 0  # Fixes slow drifts
    kd = 6.66  # Prevents "fishtailing" (wobbling)

    # PID Calculations parameters:
    Pc = 0.5  # Oscillation period from previous run
    Ns = 200  # Number of steps in loop
    Ts = 400  # Target speed in deg/s (Approx 40% power; SPIKE 3.x uses deg/s)
    Kc_initial = 2  # Proportional constant Critical value used for the run which gives noticeable oscillation but not really wild ones.
    # We will call this Kp value “Kc” (“critical gain”).

    # Windup Guard: Limits the power of the Integral term
    # This prevents the robot from over-correcting after a long drift.
    integral_limit = 200

    # -------------------------------------------------------------------------
    # SPEED AND DIRECTION SETUP
    # -------------------------------------------------------------------------

    # Convert speed percentage to hub velocity (0-100% -> 0-1000 deg/s)
    base_speed = int(speed_per * 10)

    # Direction multiplier: -1 for forward, 1 for backward
    direction = -1 if move_cm >= 0 else 1

    # Convert target distance from cm to motor degrees
    degrees_to_move = cm_to_degrees(move_cm)

    # Reset motor encoders for distance tracking
    motor.reset_relative_position(port.D, 0)
    motor.reset_relative_position(port.C, 0)

    # -------------------------------------------------------------------------
    # PID CONTROL VARIABLES
    # -------------------------------------------------------------------------
    last_error = 0
    integral = 0
    count = 0  # Loop iteration counter (for debugging)

    # -------------------------------------------------------------------------
    # PRECISION STOPPING PARAMETERS
    # -------------------------------------------------------------------------
    decel_kp = 1.2  # Deceleration coefficient (higher = earlier stop)
    min_speed = 150  # Minimum speed to overcome friction
    brake_dist = 100  # Distance (in degrees) to start slowing down (~22 cm)
    max_velocity = speed_per * 10

    # -------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # -------------------------------------------------------------------------
    while True:
        count += 1

        # --- Distance Tracking ---
        # Average position of both wheels for accuracy
        left_pos = abs(motor.relative_position(port.D))
        right_pos = abs(motor.relative_position(port.C))
        avg_pos = (left_pos + right_pos) / 2

        # Check if target distance reached
        remaining_dist = degrees_to_move - avg_pos
        if remaining_dist <= 0:
            break

        # --- Dynamic Speed Control (Proportional Ramp-Down) ---
        # Smoothly reduce speed as we approach target for precision stopping
        if remaining_dist < brake_dist:
            # Speed proportional to remaining distance, with minimum threshold
            target_velocity = max(min_speed, remaining_dist * decel_kp)
            current_base_speed = min(max_velocity, target_velocity)
        else:
            current_base_speed = max_velocity

        # --- PID Steering Correction ---
        # Read current yaw (convert decidegrees to degrees)
        current_yaw = motion_sensor.tilt_angles()[0] / 10

        # Error: deviation from straight (target yaw = 0)
        error = 0 - current_yaw

        # PID term calculations
        p_term = error * kp
        integral += error
        d_term = (error - last_error) * kd

        # Integral with windup guard
        if abs(p_term) < 300:
            integral += error
        integral = max(min(integral, integral_limit), -integral_limit)
        i_term = integral * ki

        # Combined steering correction
        steering_correction = int(p_term + i_term + d_term)
        last_error = error

        # --- Apply Motor Power ---
        # Differential steering: adjust left/right speeds to correct drift
        vel_l = (current_base_speed * direction) + steering_correction
        vel_r = (current_base_speed * direction) - steering_correction

        # Drive motors
        motor_pair.move_tank(motor_pair.PAIR_1, int(vel_l), int(vel_r))
        await runloop.sleep_ms(10)

    # -------------------------------------------------------------------------
    # STOP AND HOLD POSITION
    # -------------------------------------------------------------------------
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)

    # --- Debug Output ---
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
        "Correction: ",
        steering_correction,
    )

    # -------------------------------------------------------------------------
    # PID CALIBRATION OUTPUT (Ziegler-Nichols Method)
    # -------------------------------------------------------------------------
    # Formula: Kp=0.60*Kc ; Ki=2*Kp*dT/Pc ; Kd=Kp*Pc/(8*dT)

    await runloop.sleep_ms(100)
    end_time = time.ticks_ms()
    duration_sec = (end_time - start_time) / 1000
    print("Loop time: " + str(duration_sec))
    print("Loop iterations: ", count)
    print("Time per loop (dT): " + str(duration_sec / 500))
    end_time = time.ticks_ms()
    total_time_sec = (end_time - start_time) / 1000

    Kc = Kc_initial
    dT = total_time_sec / count  # Average time per loop iteration

    # Calculate recommended PID values using Ziegler-Nichols
    Kp_new = 0.60 * Kc
    Ki_new = 2 * Kp_new * dT / Pc
    Kd_new = Kp_new * Pc / (8 * dT)
    print("--- PID TEST RESULTS ---")
    print("Loop time: ", total_time_sec, "s")
    print("Loop iterations: ", count)
    print("Inputs: Kc=", Kc, "; dT= ", dT, "; Pc= ", Pc)
    print("--- RECOMMENDED PID PARAMS ---")
    print("Kp= ", Kp_new, "; Ki= ", Ki_new, "; Kd= ", Kd_new)


# =============================================================================
# ARM CONTROL FUNCTIONS
# =============================================================================
async def turn_right_arm(degrees_turn: int, speed_per: int):
    """
    Rotate the right arm motor (Port A) by a specified angle.

    פונקציה להזזת המנוע הימני A

    :param degrees_turn: Degrees to rotate (positive/negative for direction)
    :param speed_per: Speed as percentage (0-100%)
    """
    velocity = int(speed_per * 10)
    await motor.run_for_degrees(port.A, degrees_turn, velocity)


async def turn_left_arm(degrees_turn: int, speed_per: int):
    """
    Rotate the left arm motor (Port B) by a specified angle.

    פונקציה להזזת המנוע השמאלי B

    :param degrees_turn: Degrees to rotate (positive/negative for direction)
    :param speed_per: Speed as percentage (0-100%)
    """
    velocity = int(speed_per * 10)
    await motor.run_for_degrees(port.B, degrees_turn, velocity)


# =============================================================================
# MISSION FUNCTIONS
# =============================================================================
# Each mission function contains the sequence of movements and actions
# needed to complete specific competition tasks.
#
# To run a mission, add it to the main() function at the bottom of this file.
# =============================================================================


# -----------------------------------------------------------------------------
# MISSION 1+2
# -----------------------------------------------------------------------------
async def mission_one_and_two():
    """
    Mission 1+2: Combined mission sequence.

    Actions:
        1. Display "1+2" on hub matrix
        2. Navigate to target area with arm manipulation
        3. Execute task and return to base
    """
    print("--- Starting Mission 1+2 ---")
    await light_matrix.write("1+2")

    await turn_right_arm(30, 10)
    await turn_right_arm(-80, 10)
    await move_tank_for_cm(56)
    await turn_PID(-45)
    await move_tank_for_cm(15)
    await turn_PID(90)

    await move_tank_for_cm(6, 20)
    await turn_right_arm(-40, 10)
    await move_tank_for_cm(6, 20)

    await turn_right_arm(100, 30)
    await move_tank_for_cm(-17)
    await turn_PID(-75)
    await move_tank_for_cm(-90)


# -----------------------------------------------------------------------------
# MISSION 3+4 (+12)
# -----------------------------------------------------------------------------
async def mission_three_and_four():
    """
    Mission 3+4 (+12): Object retrieval and statue manipulation.

    Actions:
        1. Navigate to target position
        2. Retrieve object with grip mechanism
        3. Lift and manipulate statue
        4. Return to position
    """
    # --- Phase 1: Navigate to Position ---
    await move_tank_for_cm(80)
    await turn_PID(-90)
    await move_tank_for_cm(30)
    await turn_PID(-35)
    await move_tank_for_cm(-33)
    await turn_right_arm(-55, 10)
    await turn_PID(33)

    # --- Phase 2: Retrieve Object ---
    await move_tank_for_cm(5.5)  # Move Forward
    await turn_left_arm(-120, 20)  # Open grip
    await move_tank_for_cm(5)  # move forward
    await turn_left_arm(120, 20)  # Close grip
    # Lift arm up
    await turn_right_arm(70, 5)
    await runloop.sleep_ms(1000)
    # Turn arm back down
    await turn_right_arm(-100, 5)
    # Move back
    await move_tank_for_cm(-13.5)

    # --- Phase 3: Lift Statue ---
    await turn_PID(-33)
    await move_tank_for_cm(35)
    await turn_right_arm(27, 1)
    await turn_PID(20)
    await move_tank_for_cm(-3)
    await turn_PID(-30)
    await turn_left_arm(-120, 20)  # Open grip


# -----------------------------------------------------------------------------
# MISSION 5+6
# -----------------------------------------------------------------------------
async def mission_five_and_six():
    """
    Mission 5+6: Navigation sequence with multiple turns.

    Complex path navigation through the competition field.
    """
    print("--- Starting Mission 5+6 ---")

    await move_tank_for_cm(25)

    await turn_PID(40)

    await move_tank_for_cm(17)

    await turn_PID(-40)

    await move_tank_for_cm(15)

    await turn_PID(40)

    await move_tank_for_cm(4)

    # turning to target
    await turn_PID(-79)

    await move_tank_for_cm(7)

    await turn_PID(-10)

    await move_tank_for_cm(0.5)

    await turn_PID(15)

    await move_tank_for_cm(-9)

    await turn_PID(35)

    await move_tank_for_cm(11.5)

    await turn_PID(45)

    # Run Back home
    await move_tank_for_cm(-70)


# -----------------------------------------------------------------------------
# MISSION 8
# -----------------------------------------------------------------------------
# async def mission_eight(move_speed: int = 40, arm_speed: int = 50) -> None:
#     """
#     Mission 8: Arm oscillation task.

#     Actions:
#         1. Move forward to target
#         2. Oscillate arm to complete mechanism activation
#         3. Return to starting position

#     :param move_speed: Movement speed percentage (default: 40%)
#     :param arm_speed: Arm movement speed percentage (default: 50%)
#     """
#     print("--- Starting Mission 8 ---")

#     # Move forward to target position
#     await move_tank_for_cm(-37, move_speed)

#     # Oscillate right arm (4 cycles at 70 degrees)
#     await oscillate_arm(turn_right_arm, 70, arm_speed)

#     # Return to starting position
#     await move_tank_for_cm(40, move_speed)


# -----------------------------------------------------------------------------
# MISSION 9
# -----------------------------------------------------------------------------
async def missions_eight_and_nine():
    """
    Mission 9: Navigation and positioning task.

    Complex path with multiple waypoints.
    """
    print("--- Starting Missions 8+9 ---")

    await move_tank_for_cm(10)
    await turn_PID(40)
    await move_tank_for_cm(45)
    await move_tank_for_cm(-36)  # Pull back
    await move_tank_for_cm(3)
    await turn_PID(-48)
    await move_tank_for_cm(6)

    # Oscillate arm for mission 8
    for i in range(4):
        await turn_right_arm(-60, 60)
        await turn_right_arm(60, 60)

    # Move mission 8 Lever
    await move_tank_for_cm(-17)
    await turn_PID(60)
    await move_tank_for_cm(-30)


# -----------------------------------------------------------------------------
# MISSION 10
# -----------------------------------------------------------------------------
async def missions_ten_and_eleven(move_speed=40, turning_speed=40):
    """
    Missions 10 + 11: Box manipulation task.

    Actions:
        1. Navigate to jar/box location
        2. Knock over jar and grab box
        3. Return to base

    :param move_speed: Movement speed percentage (default: 40%)
    :param turning_speed: Turning speed (currently unused - uses turn_PID)
    """
    print("--- Starting Missions 10 + 11 ---")

    # --- Phase 1: Navigate to Target ---
    await move_tank_for_cm(25)

    print("Turning Left")
    await turn_PID(85)

    await move_tank_for_cm(57)

    print("Turning right")
    await turn_PID(-95)

    # --- Phase 2: Knock Over Jar and Grab Box ---
    await move_tank_for_cm(10)  # Push forward
    await move_tank_for_cm(-8)  # Pull back

    # --- Phase 3: Return to Base ---
    print("Turning right")
    await turn_PID(115)

    # await move_tank_for_cm(-70)

    for i in range(8):
        await move_tank_for_cm(-9)
        await move_tank_for_cm(9)


async def mission_twelve():
    print("--- Starting Mission 12 ---")
    await move_tank_for_cm(50)
    await move_tank_for_cm(-60)


# =============================================================================
# MAIN PROGRAM ENTRY POINT
# =============================================================================


async def main():
    """
    Main program function - orchestrates all mission sequences.

    Comment out missions you don't want to run.
    Missions are executed in order.
    """
    # Mission 1+2: Initial navigation and arm task
    await mission_one_and_two()

    # Mission 3+4: Object retrieval and statue
    await mission_three_and_four()

    # Mission 5+6: Navigation sequence
    await mission_five_and_six()

    # Mission 8: Arm oscillation task
    # Mission 9: Complex navigation
    await missions_eight_and_nine()

    # Missions 10+11: Box manipulation
    await missions_ten_and_eleven()

    # Mission 12
    await mission_twelve()


# Start the robot program
runloop.run(main())
