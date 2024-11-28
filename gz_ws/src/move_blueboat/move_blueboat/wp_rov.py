
# Markus Buchholz, 2024
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from time import sleep, time
import math
from threading import Thread
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MavlinkControlNode(Node):
    def __init__(self):
        super().__init__('mavlink_control_node')
        
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.vehicle = self.create_connection()
        self.get_logger().info("Waiting for heartbeat...")
        self.vehicle.wait_heartbeat()
        self.get_logger().info("Heartbeat received")

        self.control_thread = Thread(target=self.main_control_loop)
        self.control_thread.start()

    def create_connection(self):
        return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    def set_vehicle_mode(self, mode):
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.set_mode(mode_id)
        self.get_logger().info(f"Vehicle mode set to {mode}")

    def arm_vehicle(self):
        self.vehicle.arducopter_arm()
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle armed")

    def disarm_vehicle(self):
        self.vehicle.arducopter_disarm()
        self.vehicle.motors_disarmed_wait()
        self.get_logger().info("Vehicle disarmed")

    def fetch_current_state_local(self):
        default_position = {'x': 0, 'y': 0, 'z': 0}
        default_orientation = {'yaw': 0}

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            0, 0, 0, 0, 0, 0, 0
        )
        position_message = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        if position_message:
            position = {'x': position_message.x, 'y': position_message.y, 'z': position_message.z}
        else:
            position = default_position

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            0, 0, 0, 0, 0, 0, 0
        )
        attitude_message = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        if attitude_message:
            orientation = {'yaw': attitude_message.yaw}
        else:
            orientation = default_orientation

        return {
            'x': position['x'],
            'y': position['y'],
            'z': -position['z'],
            'yaw': orientation['yaw']
        }

    def wait_for_valid_position(self, max_attempts=2):
        for attempt in range(max_attempts):
            current_state = self.fetch_current_state_local()
            if current_state['x'] != 0 or current_state['y'] != 0 or current_state['z'] != 0:
                return current_state
            self.get_logger().warn(f"Invalid position data, retrying... ({attempt + 1}/{max_attempts})")
            sleep(1)
        raise TimeoutError("Failed to get a valid position")

    def set_guided_mode_hold_position_local(self):
        self.get_logger().info("Setting GUIDED mode and holding position...")
        current_state = self.wait_for_valid_position()

        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode
        )

        self.vehicle.mav.set_position_target_local_ned_send(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            current_state['x'],
            current_state['y'],
            current_state['z'],
            0, 0, 0,
            0, 0, 0,
            current_state['yaw'],
            0
        )
        return current_state

    def send_position_request(self, x, y, z):
        type_mask = 0b0000111111111000  # Only positions
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        self.vehicle.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.vehicle.target_system,  # Target system
            self.vehicle.target_component,  # Target component
            coordinate_frame,  # Coordinate frame
            type_mask,  # Type mask
            x, y, z,  # Positions
            0, 0, 0,  # Velocities (not used)
            0, 0, 0,  # Accelerations (not used)
            0, 0  # Yaw and yaw rate (not used)
        )
        self.get_logger().info(f"Position request sent: x={x}, y={y}, z={z}")

    def has_reached_position(self, current, target, threshold=1.0):
        self.get_logger().debug(f"Current Position: x={current['x']}, y={current['y']}, z={current['z']}")
        self.get_logger().debug(f"Target Position: x={target['x']}, y={target['y']}, z={target['z']}")

        distance = math.sqrt((current['x'] - target['x'])**2 + (current['y'] - target['y'])**2 + (current['z'] - target['z'])**2)
        self.get_logger().info(f"Distance to target: {distance} (Threshold: {threshold})")

        return distance < threshold

    def main_control_loop(self):
        try:
            current_state = self.set_guided_mode_hold_position_local()

            self.arm_vehicle()

            # define waypoints (x, y, z) in 3D space
            waypoints = [
                {'x': current_state['x'] + 1.0, 'y': current_state['y'], 'z': current_state['z'] - 2.0},
                {'x': current_state['x'] + 2.0, 'y': current_state['y'] + 2.0, 'z': current_state['z'] - 2.0},
                {'x': current_state['x'] + 3.0, 'y': current_state['y'] - 2.0, 'z': current_state['z'] - 4.0},
                {'x': current_state['x'] + 4.0, 'y': current_state['y'], 'z': current_state['z'] - 4.0},
                {'x': current_state['x'] + 5.0, 'y': current_state['y'] + 2.0, 'z': current_state['z'] - 6.0}
            ]

            for waypoint in waypoints:
                self.send_position_request(waypoint['x'], waypoint['y'], -waypoint['z'])

                start_time = time()
                while True:
                    current_position = self.fetch_current_state_local()
                    if self.has_reached_position(current_position, waypoint):
                        self.get_logger().info(f"Reached waypoint: x={waypoint['x']}, y={waypoint['y']}, -z={waypoint['z']}")
                        break
                    sleep(0.5) 

            # switch to ALT_HOLD mode after reaching the last waypoint - "rov stablizied"
            self.set_vehicle_mode('ALT_HOLD')
            self.get_logger().info("Vehicle switched to ALT_HOLD mode")

            current_position = self.fetch_current_state_local()
            self.get_logger().info(f"Final Position: x={current_position['x']}, y={current_position['y']}, z={current_position['z']}")

            # Disarm the vehicle
            self.disarm_vehicle()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            self.disarm_vehicle()

def main(args=None):
    rclpy.init(args=args)
    mavlink_node = MavlinkControlNode()
    
    try:
        rclpy.spin(mavlink_node)
    except KeyboardInterrupt:
        pass
    finally:
        mavlink_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
