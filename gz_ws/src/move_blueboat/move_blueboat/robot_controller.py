import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

def quaternion_to_euler(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.motor_port_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.motor_stbd_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)
        self.odometry_subscriber = self.create_subscription(Odometry, '/model/blueboat/odometry', self.odometry_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        port_thrust = Float64()
        stbd_thrust = Float64()

        # Set your thrust values here
        port_thrust.data = 1.0
        stbd_thrust.data = 5.0

        # Publish thrust commands
        self.motor_port_publisher.publish(port_thrust)
        self.motor_stbd_publisher.publish(stbd_thrust)

    def odometry_callback(self, msg):
        # Extract position
        position = msg.pose.pose.position
        pos_str = f"Position - x: {position.x}, y: {position.y}, z: {position.z}"

        # Extract orientation as a Quaternion
        orientation_q = msg.pose.pose.orientation

        # Convert Quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(orientation_q)
        ori_str = f"Orientation - roll: {math.degrees(roll)}, pitch: {math.degrees(pitch)}, yaw: {math.degrees(yaw)}"

        # Log position and orientation
        self.get_logger().info(f"{pos_str}, {ori_str}")

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
