import time
from pymavlink import mavutil

def connect_vehicle():
    print("Connecting to vehicle...")
    vehicle = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    vehicle.wait_heartbeat()
    print("Connected to vehicle.")
    return vehicle

def set_guided_mode(vehicle):
    print("Setting GUIDED mode...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_GUIDED_ARMED,  # mode
        0, 0, 0, 0, 0, 0  # param1 ~ param7 (not used)
    )

    while True:
        ack_msg = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack_msg:
            ack_type = mavutil.mavlink.enums['MAV_RESULT'][ack_msg.result].name
            print(f"ACK received: {ack_type}")
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("GUIDED mode accepted.")
                break
            else:
                print(f"GUIDED mode not accepted, result: {ack_msg.result}")
        else:
            print("No ACK received for GUIDED mode command, retrying...")
            vehicle.mav.command_long_send(
                vehicle.target_system,
                vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_GUIDED_ARMED,  # mode
                0, 0, 0, 0, 0, 0  # param1 ~ param7 (not used)
            )
            time.sleep(1)
    
    print("Vehicle is now in GUIDED mode.")

def arm_vehicle(vehicle):
    print("Arming vehicle...")
    vehicle.arducopter_arm()
    while not vehicle.motors_armed():
        print("Waiting for arming...")
        time.sleep(1)
    print("Vehicle is armed.")

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    print("Called send_ned_velocity")
    msg = vehicle.mav.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    )
    vehicle.mav.send(msg)

def main():
    vehicle = connect_vehicle()
    set_guided_mode(vehicle)
    arm_vehicle(vehicle)
    
    duration = 5  # seconds
    velocity_x = 0.5
    velocity_y = 0.5
    velocity_z = 0.0

    print("Starting velocity command loop")
    start_time = time.time()
    while time.time() - start_time < duration:
        print(f"Looping... {time.time() - start_time:.2f} seconds elapsed")
        send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z)
        time.sleep(1)
    
    print("Velocity command completed.")

if __name__ == "__main__":
    main()
