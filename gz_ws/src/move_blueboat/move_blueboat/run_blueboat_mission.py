#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
import math
import yaml
from scipy.interpolate import CubicSpline

def quaternion_to_yaw(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)
        self.odometry_subscriber = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10)
        
        self.Kp = 0.25
        self.Ki = 0.0
        self.Kd = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.constant_forward_speed = 0.25 / 1

        waypoints = self.load_waypoints_from_yaml('waypoints_asv.yaml')
        self.smooth_x, self.smooth_y = self.generate_spline_path(waypoints)

    def load_waypoints_from_yaml(self, file_path):
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            waypoints_data = yaml_data['waypoints']
            waypoints = [ [wp['point'][0], wp['point'][1]] for wp in waypoints_data ]
            return np.array(waypoints)

    def generate_spline_path(self, waypoints):
        x = waypoints[:, 0]
        y = waypoints[:, 1]
        distance = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0)
        spline_x = CubicSpline(distance, x)
        spline_y = CubicSpline(distance, y)
        fine_distance = np.linspace(0, distance[-1], 1000)
        return spline_x(fine_distance), spline_y(fine_distance)

    def odometry_callback(self, data):
        current_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        current_yaw = quaternion_to_yaw(data.pose.pose.orientation)
        self.control_robot(current_position, current_yaw)
        self.log_position_and_yaw(current_position, current_yaw)

    def control_robot(self, current_position, current_yaw):
        distances = np.sqrt((self.smooth_x - current_position[0])**2 + (self.smooth_y - current_position[1])**2)
        closest_index = np.argmin(distances)

        if closest_index >= len(self.smooth_x) - 1:
            self.publish_thrust(0, 0)
            self.get_logger().info("Last waypoint reached. Stopping the robot.")
            return

        lookahead_distance = 1.0 * 1
        cumulative_distances = np.cumsum(np.sqrt(np.diff(self.smooth_x)**2 + np.diff(self.smooth_y)**2))
        lookahead_index = np.searchsorted(cumulative_distances, cumulative_distances[closest_index] + lookahead_distance)
        lookahead_index = min(lookahead_index, len(self.smooth_x) - 1)

        lookahead_point = (self.smooth_x[lookahead_index], self.smooth_y[lookahead_index])

        los_angle = math.atan2(lookahead_point[1] - current_position[1], lookahead_point[0] - current_position[0])
        heading_error = math.atan2(math.sin(los_angle - current_yaw), math.cos(los_angle - current_yaw))

        self.integral_error += heading_error
        derivative_error = heading_error - self.prev_error

        thrust_adjustment = self.Kp * heading_error + self.Ki * self.integral_error + self.Kd * derivative_error
        self.prev_error = heading_error
        thrust_scale_factor = 10
        # try:
        #     thrust_scale_factor = 1  # Adjust this scale factor as needed
        #     left_thrust = (self.constant_forward_speed - thrust_adjustment)* thrust_scale_factor
        #     right_thrust = (self.constant_forward_speed + thrust_adjustment)* thrust_scale_factor

        # except Exception as e:
        #     self.get_logger().error(f"Error in thrust calculation: {e}")
        #     left_thrust, right_thrust = 0.0, 0.0
        left_thrust = max(min(self.constant_forward_speed - thrust_adjustment, 1.0), 0.0)
        right_thrust = max(min(self.constant_forward_speed + thrust_adjustment, 1.0), 0.0)

        self.publish_thrust(left_thrust * thrust_scale_factor, right_thrust* thrust_scale_factor)
        self.log_thrust_values(left_thrust* thrust_scale_factor, right_thrust* thrust_scale_factor)

    def publish_thrust(self, left_thrust, right_thrust):
        # Check if the thrust values are floats
        if not isinstance(left_thrust, float) or not isinstance(right_thrust, float):
            self.get_logger().error(f"Invalid thrust values: Left: {left_thrust}, Right: {right_thrust}")
            return

        port_thrust = Float64()
        stbd_thrust = Float64()
        port_thrust.data = left_thrust
        stbd_thrust.data = right_thrust
        self.motor_port_publisher.publish(port_thrust)
        self.motor_stbd_publisher.publish(stbd_thrust)

    def log_position_and_yaw(self, position, yaw):
        self.get_logger().info(f"Current position: x={position[0]}, y={position[1]}, Current yaw: {math.degrees(yaw):.2f} degrees")

    def log_thrust_values(self, left_thrust, right_thrust):
        self.get_logger().info(f"Left Thrust: {left_thrust}, Right Thrust: {right_thrust}")

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
