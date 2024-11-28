from pymavlink import mavutil
import time

def send_waypoints(waypoints):
    # Connect to the vehicle
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    
    # Wait for a heartbeat to confirm the connection
    master.wait_heartbeat()
    print("Heartbeat from vehicle received.")

    # Arm
# master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

    # Clear any existing mission and set the new mission count
    master.waypoint_clear_all_send()
    master.waypoint_count_send(len(waypoints))
    
    for seq, waypoint in enumerate(waypoints):
        # Define waypoint
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        current = 2 if seq == 0 else 0  # First waypoint is current
        autocontinue = 1  # Autocontinue to next waypoint
        param1, param2, param3, param4 = 0, 0, 0, 0  # Hold time, acceptance radius, etc.
        lat, lon, alt = waypoint  # Unpack the waypoint
        
        # Wait for the vehicle to request each waypoint, then send it
        msg = master.recv_match(type='MISSION_REQUEST', blocking=True)
        print(f"Mission request for seq {seq} received.")
        
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            seq,
            frame,
            command,
            current,
            autocontinue,
            param1, param2, param3, param4,
            lat, lon, alt
        )
    
    # Wait for mission acknowledgement
    msg = master.recv_match(type='MISSION_ACK', blocking=True)
    print("Mission acknowledged by vehicle.")

if __name__ == "__main__":
    # Example waypoints in lat, lon, alt format
    waypoints = [
        (2.0, 1.0, -1),
        (3.0, 1.0, -2),
        (4.0, 1.0, -3),
        (5.0, 1.0, -4)
    ]
    
    send_waypoints(waypoints)
