import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
import math
import time


class ASVController(Node):
    def __init__(self):
        super().__init__('asv_controller')

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)
        self.navsat_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)
        self.odometry_subscription = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10)
        self.asv_pos_gps_publisher = self.create_publisher(PointStamped, '/asv_pos_gps', 10)

        self.reference_position = (-22.986686999936378, -43.2023544348319)
        #self.waypoints_from_planner = self.load_waypoints_from_file()
        #self.waypoints = [self.map_coordinates(x, y) for x, y in self.waypoints_from_planner]
        self.waypoints = [(5, 5), (5, 0),(0, 0)] #, (20, 15), (20, 10), (20, 5), (20, 0), (10, 0), (0, 0)]

        self.print = self.on_terminal(self.waypoints)

        # PID parameters for linear movement
        self.linear_kP = 0.5
        self.linear_kI = 0.1
        self.linear_kD = 0.05

        # PID parameters for angular movement
        self.angular_kP = 3.0  # Increased from 1.5
        self.angular_kI = 0.3  # Increased from 0.1
        self.angular_kD = 0.2  # Increased from 0.05

        self.current_position = (0, 0)
        self.current_yaw = 0.0
        self.state = 'rotate_to_waypoint'

        self.current_waypoint_index = 0

        # Initialize PID errors and timestamps
        self.linear_integral = 0.0
        self.previous_linear_error = 0.0
        self.angular_integral = 0.0
        self.previous_angular_error = 0.0
        self.previous_time = time.time()

    def on_terminal(self, waypoints):
        for x, y in waypoints:
            print(x, ", ", y)

    def map_coordinates(self, x, y):
        new_x = (y + 5) * (10 / 10)
        new_y = (x + 5) * (-10 / 10) + 5
        return new_x, new_y

    def load_waypoints_from_file(self, filename='mission_path.txt'):
        waypoints = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    line = line.strip()
                    if line:
                        x, y = map(float, line.split(','))
                        waypoints.append((x / 10, y / 10))
        except FileNotFoundError:
            self.get_logger().error(f"File {filename} not found.")
        except Exception as e:
            self.get_logger().error(f"Error reading file {filename}: {e}")
        return waypoints

    def navsat_callback(self, msg):
        current_position_gps = (msg.latitude, msg.longitude)
        self.current_position = self.convert_gps_to_xy(current_position_gps)

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = float(self.current_position[0])
        pos_msg.point.y = float(self.current_position[1])

        self.asv_pos_gps_publisher.publish(pos_msg)
        self.navigate_to_waypoint()

    def odometry_callback(self, msg):
        orientation_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.current_yaw = yaw

    def convert_gps_to_xy(self, gps_position):
        ref_lat, ref_lon = self.reference_position
        lat, lon = gps_position

        delta_lat = lat - ref_lat
        delta_lon = lon - ref_lon

        meters_per_degree_lat = 111320
        meters_per_degree_lon = meters_per_degree_lat * math.cos(math.radians(ref_lat))

        x = delta_lon * meters_per_degree_lon
        y = delta_lat * meters_per_degree_lat

        return x, y

    def navigate_to_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_asv()
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        distance_to_waypoint = self.calculate_distance(self.current_position, waypoint)
        bearing_to_waypoint = self.calculate_bearing(self.current_position, waypoint)
        heading_error = self.normalize_angle(bearing_to_waypoint - self.current_yaw)

        self.get_logger().info(f"State: {self.state}, Current Position: {self.current_position}, Target Waypoint: {waypoint}, Distance Left: {distance_to_waypoint:.2f} meters, Heading Error: {heading_error:.2f}")

        current_time = time.time()
        delta_time = current_time - self.previous_time

        if self.state == 'rotate_to_waypoint':
            if abs(heading_error) < 0.1:
                self.state = 'move_to_waypoint'
                self.get_logger().info("Transition to state: move_to_waypoint")
            else:
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(0.0, angular_velocity)
        elif self.state == 'move_to_waypoint':
            if distance_to_waypoint < 1.0:
                self.state = 'stop_at_waypoint'
                self.stop_asv()
                self.get_logger().info("Transition to state: stop_at_waypoint")
            else:
                linear_velocity = self.calculate_pid(self.linear_kP, self.linear_kI, self.linear_kD, distance_to_waypoint, self.previous_linear_error, self.linear_integral, delta_time)
                angular_velocity = self.calculate_pid(self.angular_kP, self.angular_kI, self.angular_kD, heading_error, self.previous_angular_error, self.angular_integral, delta_time)
                self.publish_twist(linear_velocity, angular_velocity)
        elif self.state == 'stop_at_waypoint':
            self.stop_asv()
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.state = 'rotate_to_waypoint'
                self.get_logger().info("Transition to state: rotate_to_waypoint")
            else:
                self.state = 'idle'
                self.get_logger().info("All waypoints achieved, state: idle")

        self.previous_time = current_time

    def calculate_pid(self, kP, kI, kD, error, previous_error, integral, delta_time):
        integral += error * delta_time
        derivative = (error - previous_error) / delta_time
        output = kP * error + kI * integral + kD * derivative
        return output

    def publish_twist(self, linear_x, angular_z):
        thrust_port = linear_x - angular_z
        thrust_stbd = linear_x + angular_z

        max_thrust = 10.0
        thrust_port = max(min(thrust_port, max_thrust), -max_thrust)
        thrust_stbd = max(min(thrust_stbd, max_thrust), -max_thrust)

        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

        self.get_logger().info(f"Publishing thrust: Port={thrust_port}, Starboard={thrust_stbd}")

    def stop_asv(self):
        self.publish_twist(0.0, 0.0)

    @staticmethod
    def euler_from_quaternion(quat):
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw


    @staticmethod
    def calculate_distance(pointA, pointB):
        return math.sqrt((pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)

    @staticmethod
    def calculate_bearing(pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        angle = math.atan2(y2 - y1, x2 - x1)
        return angle

    @staticmethod
    def normalize_angle(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ASVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
