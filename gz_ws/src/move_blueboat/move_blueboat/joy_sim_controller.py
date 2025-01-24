# Copyright 2024, Markus Buchholz

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import threading
import time

class BlueBoatJoystickController(Node):

    def __init__(self):
        super().__init__('blue_boat_joystick_controller')

        self.callback_group = ReentrantCallbackGroup()
        
        self.motor_port_publisher = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_port_joint/cmd_thrust',
            10,
            callback_group=self.callback_group
        )
        
        self.motor_stbd_publisher = self.create_publisher(
            Float64,
            '/model/blueboat/joint/motor_stbd_joint/cmd_thrust',
            10,
            callback_group=self.callback_group
        )

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.handle_joy,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        self.port_thrust = 0.0
        self.stbd_thrust = 0.0

        self.lock = threading.Lock()

        self.get_logger().info('BlueBoatJoystickController initialized')

        self.running_event = threading.Event()
        self.running_event.set()

        self.factor = 15.0  # check with you simulation and need. 

        self.update_thread = threading.Thread(target=self.update_thrust)
        self.update_thread.start()


    def update_thrust(self):
        while self.running_event.is_set():
            with self.lock:
                port_thrust_value = self.port_thrust * self.factor
                stbd_thrust_value = self.stbd_thrust * self.factor
                self.motor_port_publisher.publish(Float64(data=port_thrust_value))
                self.motor_stbd_publisher.publish(Float64(data=stbd_thrust_value))
                self.get_logger().info(f'port thrust: {port_thrust_value}, stbd thrust: {stbd_thrust_value}')
            time.sleep(0.1)

    def handle_joy(self, msg):
        with self.lock:
            # axis 1  - handler left - for left/right
            # axis 4 - handler right - for forward/backward
            forward_backward = msg.axes[4]  
            left_right = msg.axes[0]  

            self.port_thrust = max(min(forward_backward + left_right, 1.0), -1.0)
            self.stbd_thrust = max(min(forward_backward - left_right, 1.0), -1.0)

    def destroy_node(self):
        self.running_event.clear()
        self.update_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    blue_boat_joystick_controller = BlueBoatJoystickController()

    def on_closing():
        blue_boat_joystick_controller.destroy_node()
        rclpy.shutdown()

    try:
        rclpy.spin(blue_boat_joystick_controller)
    except KeyboardInterrupt:
        pass
    finally:
        on_closing()

if __name__ == '__main__':
    main()
