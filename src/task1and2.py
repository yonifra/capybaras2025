from hub import light_matrix, port
import runloop
import motor_pair

async def main():
    # Display start message
    await light_matrix.write("GO!")
    
    # Configure motor pair - adjust ports based on your robot setup
    # Common configurations: ports C & D, or ports E & F
    # Try: port.C (left) and port.D (right)
    motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)
    
    # 1. Go forward 70cm
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 70 * 10, 0, velocity=500)    # 2. Turn 45 degrees to the left
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 225, -100, velocity=300)

    # 3. Move forward 20cm
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 20 * 10, 0, velocity=500)

    # 4. Go back 20cm
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -20 * 10, 0, velocity=500)

    # 5. Turn right 45 degrees
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 225, 100, velocity=300)

    # 6. Go backwards 70cm
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -70 * 10, 0, velocity=500)

    # Display completion message
    await light_matrix.write("DONE")

runloop.run(main())
