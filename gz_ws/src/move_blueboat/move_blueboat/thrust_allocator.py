import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic
import math

class ASVController(Node):
    def __init__(self):
        super().__init__('asv_controller')

        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.navsat_subscription = self.create_subscription(NavSatFix, '/navsat', self.navsat_callback, 10)

        self.asv_pos_gps_publisher = self.create_publisher(PointStamped, '/asv_pos_gps', 10)

        self.reference_position = (-22.986686999936378, -43.2023544348319, -11765.090082142502)

    def cmd_vel_callback(self, msg):

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # compute the thrust for each motor
        # assuming a simple differential thrust model
        k_linear = 1.0  # gain for linear velocity
        k_angular = 1.0  # gain for angular velocity

        thrust_port = k_linear * linear_x - k_angular * angular_z
        thrust_stbd = k_linear * linear_x + k_angular * angular_z

        port_thrust_msg = Float64()
        stbd_thrust_msg = Float64()
        port_thrust_msg.data = thrust_port
        stbd_thrust_msg.data = thrust_stbd

        self.motor_port_publisher.publish(port_thrust_msg)
        self.motor_stbd_publisher.publish(stbd_thrust_msg)

    def navsat_callback(self, msg):
        current_position = (msg.latitude, msg.longitude, msg.altitude)

        distance = geodesic(self.reference_position[:2], current_position[:2]).meters
        bearing = self.calculate_bearing(self.reference_position[:2], current_position[:2])

        #self.get_logger().info(f'Distance from reference: {distance} meters, Bearing: {bearing} degrees')

        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point.x = distance * math.cos(math.radians(bearing))  # Easting
        pos_msg.point.y = distance * math.sin(math.radians(bearing))  # Northing
        pos_msg.point.z = current_position[2] - self.reference_position[2]  # Altitude difference

        self.asv_pos_gps_publisher.publish(pos_msg)

    @staticmethod
    def calculate_bearing(pointA, pointB):

        lat1, lon1 = map(math.radians, pointA)
        lat2, lon2 = map(math.radians, pointB)

        d_lon = lon2 - lon1

        x = math.sin(d_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(d_lon))

        initial_bearing = math.atan2(x, y)

        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing

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
