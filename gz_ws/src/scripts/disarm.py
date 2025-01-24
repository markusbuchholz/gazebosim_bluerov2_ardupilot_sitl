from pymavlink import mavutil
import time

# Create the connection to the vehicle
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the first heartbeat to ensure the connection is established
master.wait_heartbeat()
print("Connection established!")

# Send disarm command
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # Confirmation
    0,  # Param1: 0 to disarm
    0, 0, 0, 0, 0, 0)

# Check if the vehicle has disarmed
print("Disarming the vehicle...")
timeout = time.time() + 10  # 10 seconds timeout
disarmed = False

while not disarmed and time.time() < timeout:
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
    if heartbeat:
        disarmed = not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if disarmed:
            print('Vehicle is disarmed!')
            break

if not disarmed:
    print('Failed to disarm the vehicle. Check vehicle status and conditions.')
