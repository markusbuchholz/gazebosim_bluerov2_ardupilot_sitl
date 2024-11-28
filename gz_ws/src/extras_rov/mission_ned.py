from pymavlink import mavutil
import time

def send_local_ned_positions(positions):
    # Connect to the vehicle
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    
    # Wait for a heartbeat to confirm the connection
    master.wait_heartbeat()
    print("Heartbeat from vehicle received.")

    # Arm the vehicle
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # 1 to arm
        0, 0, 0, 0, 0, 0
    )

    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

    # Send NED position targets
    for position in positions:
        # Unpack NED coordinates
        north, east, down = position

        # Ensure time_boot_ms is within the unsigned 32-bit integer range
        time_boot_ms = int(time.time() * 1000) % 4294967295
        
        # Send SET_POSITION_TARGET_LOCAL_NED command
        master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask=(1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7),  # Position + Altitude
            x=north,
            y=east,
            z=down,
            vx=0,
            vy=0,
            vz=0,
            afx=0,
            afy=0,
            afz=0,
            yaw=0,
            yaw_rate=0
        )
        
        print(f"Sent NED position: North: {north}, East: {east}, Down: {down}")
        time.sleep(1)  # Adjust based on how quickly you want to send the next command

if __name__ == "__main__":
    # Example NED positions (in meters)
    positions = [
        (0, 0, -5),  # Move 5 meters down
        (5, 0, -5),  # Move 5 meters north and maintain depth
        (5, 5, -5),  # Move 5 meters east and maintain depth
        (0, 5, -5)   # Return to just above the original position
    ]
    
    send_local_ned_positions(positions)
