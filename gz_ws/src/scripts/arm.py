from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Try to arm the vehicle
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# Wait and check for arming confirmation
print("Attempting to arm the vehicle...")
timeout = time.time() + 10  # 10 second timeout for arming
armed = False

while not armed and time.time() < timeout:
    message = master.recv_match(type='HEARTBEAT', blocking=True)
    if message:
        print(message.to_dict())
        armed = message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            print('Vehicle is armed!')
            break

if not armed:
    print('Failed to arm the vehicle. Check pre-arm conditions and messages.')
