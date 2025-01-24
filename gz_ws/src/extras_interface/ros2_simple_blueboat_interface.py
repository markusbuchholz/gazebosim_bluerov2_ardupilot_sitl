#!/usr/bin/env python3
# Markus Buchholz, 2025 (Modified for Blueboat)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Int32MultiArray, Float32  # Import for PWM and Float32 data

from pymavlink import mavutil
import threading
import time
import numpy as np

class BlueboatROSInterface(Node):
    def __init__(self):
        super().__init__('blueboat_ros_interface')

        # Declare parameters
        self.declare_parameter('mavlink_connection', 'udpin:0.0.0.0:14550')
        self.declare_parameter('data_stream_rate', 8)  
        self.declare_parameter('num_thrusters', 8)
        self.declare_parameter('uuv_name', 'blueboat')  # Added parameter for uuv_name

        # Retrieve parameters
        mavlink_connection_str = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        data_stream_rate = self.get_parameter('data_stream_rate').get_parameter_value().integer_value
        self.num_thrusters = self.get_parameter('num_thrusters').get_parameter_value().integer_value
        self.uuv_name = self.get_parameter('uuv_name').get_parameter_value().string_value  # Initialize uuv_name

        self.get_logger().info(f'Connecting to ArduPilot via {mavlink_connection_str}...')
        self.conn = mavutil.mavlink_connection(mavlink_connection_str)
        self.conn.wait_heartbeat()
        self.get_logger().info(f'Heartbeat from system (system {self.conn.target_system} component {self.conn.target_component})')

        self.thrusterRanges = [1100.0, 1900.0]  # PWM range for thrusters
        self.thrusterInputRanges = [-3.0, 3.0]   # Control input range

        # Backup and set modes
        # self.backup_params = self.backup_thruster_params()

        self.set_stabilize_mode_and_arm()
        #self.enable_passthrough_mode()

        # Define QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for servo outputs
        self.servo_output_publisher = self.create_publisher(
            Int32MultiArray,
            '/blueboat/servo_outputs',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )

        # Subscription for cmd_vel
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/blueboat/cmd_vel',
            self._cmd_vel_callback,
            qos_profile_best_effort,
            callback_group=ReentrantCallbackGroup()
        )

        # Define custom QoS for motor thrust topics
        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions for motor thrust
        self.motor_port_subscription = self.create_subscription(
            Float32,
            f'{self.uuv_name}/send_port_motor_0_100_thrust',
            self.motor_port_thrust_callback,
            self.custom_qos,
            callback_group=ReentrantCallbackGroup()
        )

        self.motor_stbd_subscription = self.create_subscription(
            Float32,
            f'{self.uuv_name}/send_stbd_motor_0_100_thrust',
            self.motor_stbd_thrust_callback,
            self.custom_qos,
            callback_group=ReentrantCallbackGroup()
        )

        self.data_lock = threading.Lock()
        self.data = {}

        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        self.timer = self.create_timer(1.0 / 30.0, self._publish_servo_outputs)  # 30 Hz

        self.get_logger().info('Blueboat ROS Interface node initialized.')

    def backup_thruster_params(self):
        """Backup the current SERVO_FUNCTION parameters."""
        backup = []
        for i in range(1, self.num_thrusters + 1):
            param_name = f'SERVO{i}_FUNCTION'
            self.conn.mav.param_request_read_send(
                self.conn.target_system,
                self.conn.target_component,
                param_name.encode('utf-8'),
                -1
            )
            message = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if message:
                backup.append(int(message.param_value))
                self.get_logger().info(f'Backed up {param_name}: {int(message.param_value)}')
            else:
                backup.append(0)
                self.get_logger().warn(f'Failed to backup {param_name}, defaulting to 0')
        return backup

    def set_servo_function(self, servo_num, function):
        """Set the SERVO_FUNCTION parameter for a specific thruster."""
        param_name = f'SERVO{servo_num}_FUNCTION'
        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            param_name.encode('utf-8'),
            function,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        self.get_logger().info(f'Set {param_name} to {function}')
        time.sleep(0.1)  # Small delay to ensure the command is processed

    def enable_passthrough_mode(self):
        """Enable passthrough mode for all thrusters."""
        self.get_logger().info('Enabling passthrough mode for thrusters...')
        for i in range(1, self.num_thrusters + 1):
            self.set_servo_function(i, 1)  # 1 typically stands for passthrough
        self.get_logger().info('Passthrough mode enabled.')

    def disable_passthrough_mode(self):
        """Disable passthrough mode and restore original thruster functions."""
        self.get_logger().info('Disabling passthrough mode and restoring thruster functions...')
        for i in range(1, self.num_thrusters + 1):
            original_function = self.backup_params[i-1]
            self.set_servo_function(i, original_function)
        self.get_logger().info('Passthrough mode disabled and thruster functions restored.')

    def set_stabilize_mode_and_arm(self):
        """Set the vehicle's mode to STABILIZE and arm it."""
        # self.get_logger().info('Setting STABILIZE mode...')
        # self.conn.mav.set_mode_send(
        #     self.conn.target_system,
        #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #     0  # 
        # )
        time.sleep(1)  # Wait for mode to be set

        self.get_logger().info('Arming the vehicle...')
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # Arm
        )
        
        armed = False
        for _ in range(10):
            message = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if message and (message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                self.get_logger().info('Vehicle armed successfully.')
                break
            time.sleep(0.5)
        if not armed:
            self.get_logger().error('Failed to arm the vehicle.')
            raise Exception('Vehicle failed to arm.')

    def mavlink_listener(self):
        """Continuously listen for MAVLink messages and update data."""
        while rclpy.ok():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
                if msg:
                    msg_type = msg.get_type()
                    with self.data_lock:
                        self.data[msg_type] = msg.to_dict()
                    self.get_logger().debug(f'Received MAVLink message: {msg_type}')
            except Exception as e:
                self.get_logger().error(f'Error while receiving MAVLink message: {e}')
                time.sleep(0.1)

    def _publish_servo_outputs(self):
        """Extract SERVO_OUTPUT_RAW data and publish PWM values."""
        with self.data_lock:
            servo_output = self.data.get('SERVO_OUTPUT_RAW', None)

        if not servo_output:
            self.get_logger().warn('SERVO_OUTPUT_RAW message not received yet.')
            return

        # Extract servo1_raw to servo8_raw
        pwm_values = [
            servo_output.get('servo1_raw', 0),
            servo_output.get('servo2_raw', 0),
            servo_output.get('servo3_raw', 0),
            servo_output.get('servo4_raw', 0),
            servo_output.get('servo5_raw', 0),
            servo_output.get('servo6_raw', 0),
            servo_output.get('servo7_raw', 0),
            servo_output.get('servo8_raw', 0)
        ]

        pwm_msg = Int32MultiArray()
        pwm_msg.data = pwm_values
        self.servo_output_publisher.publish(pwm_msg)

        self.get_logger().debug(f'Published PWM Values: {pwm_values}')

    def mapRanges(self, value):
        """Map a value from thruster input range to PWM range and clip."""
        in_min, in_max = self.thrusterInputRanges
        out_min, out_max = self.thrusterRanges

        # Perform linear mapping
        mapped_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        # Clip the mapped value to thrusterRanges
        clipped_value = np.clip(mapped_value, out_min, out_max)

        return clipped_value

    def _cmd_vel_callback(self, msg):
        """Handle incoming velocity commands and translate to RC overrides for Blueboat."""
        override = []

        try:
            # Example usage: 
            #   ros2 topic pub /blueboat/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

            override.append(1500 - msg.linear.y)        # Example pitch
            override.append(int(self.mapRanges(msg.angular.x)))  # Ensure integer values
            override.append(int(self.mapRanges(msg.linear.z)))
            override.append(int(3000 - self.mapRanges(msg.angular.z)))  # Example forward
            override.append(int(self.mapRanges(msg.linear.x)))
            override.append(65535)  # Lateral
            override.append(65535)  # Light strength
            override.append(65535)  # Camera servo tilt

            while len(override) < 8:
                override.append(65535)

            self.get_logger().debug(f'Sending RC Channels Override: {override}')
            self.set_rc_channels_pwm(override)
            self.get_logger().info(f'RC Channels Override sent: {override}')
        except Exception as e:
            self.get_logger().error(f'Failed to process cmd_vel: {e}')

    def set_rc_channels_pwm(self, vals):
        """Override RC channels with provided PWM values."""
        rc_channel_values = [int(val) for val in vals[:8]]
        while len(rc_channel_values) < 8:
            rc_channel_values.append(65535)  # Typically 65535 means no change

        self.get_logger().debug(f'Sending RC Channels Override: {rc_channel_values}')

        try:
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,      # target_system
                self.conn.target_component,   # target_component
                *rc_channel_values
            )
            self.get_logger().info(f'RC Channels Override sent: {rc_channel_values}')
        except Exception as e:
            self.get_logger().error(f'Failed to send RC override: {e}')

    # New Callback for Port Motor Thrust
    def motor_port_thrust_callback(self, msg):
        """Handle incoming port motor thrust commands."""
        thrust_value = int(msg.data)
        self.get_logger().info(f'Received port motor thrust: {thrust_value}')
        self.send_motor_thrust(1, thrust_value)  # Assuming motor number 3 for port

    # New Callback for Starboard Motor Thrust
    def motor_stbd_thrust_callback(self, msg):
        """Handle incoming starboard motor thrust commands."""
        thrust_value = int(msg.data)
        self.get_logger().info(f'Received starboard motor thrust: {thrust_value}')
        self.send_motor_thrust(2, thrust_value)  # Assuming motor number 4 for starboard

    def send_motor_thrust(self, motor_number, thrust_value):
        """
        Sends a MAV_CMD_DO_MOTOR_TEST command to the vehicle.

        :param motor_number: Motor number (1-8 for an 8-thruster vehicle)
        :param thrust_value: Thrust value (0-100 for percent)
        """
        thrust_type = 0  # 0 = Percent
        duration = 1.0 / 1.0  # Duration per cycle based on 30Hz

        self.get_logger().info(f'Sending MOTOR_TEST command: Motor {motor_number}, Thrust {thrust_value}%')

        try:
            self.conn.mav.command_long_send(
                self.conn.target_system,          # target_system
                self.conn.target_component,       # target_component
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,  # command
                0,                                # confirmation
                motor_number,                     # param1: motor number
                thrust_type,                      # param2: thrust type
                thrust_value,                     # param3: thrust value
                duration,                         # param4: duration
                0, 0, 0                           # param5-7: unused
            )
            self.get_logger().info(f'MOTOR_TEST command sent to motor {motor_number} with thrust {thrust_value}%')
        except Exception as e:
            self.get_logger().error(f'Failed to send MOTOR_TEST command: {e}')

    def shutdown(self):
        """Safely shutdown the node, disarm the vehicle, and disable passthrough."""
        self.get_logger().info('Shutting down Blueboat ROS Interface node...')
        try:
            # Disarm the vehicle
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0  # Disarm
            )
            self.get_logger().info('Vehicle disarmed.')

            # Disable passthrough mode if previously enabled
            # self.disable_passthrough_mode()

            # Close MAVLink connection
            self.conn.close()
            self.get_logger().info('MAVLink connection closed.')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BlueboatROSInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Initiating shutdown...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
