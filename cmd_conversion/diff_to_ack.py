import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

WHEEL_BASE = 0.550 # m
FWD_VEL_LOW = 1e-3 # m/s
ANG_VEL_LOW = 1e-6 # rad/s
DELTA_MAX = 0.69   # rad
PUBLISH_FREQ = 1   # Hz
FWD_VEL_DIFF = 0.5
ANG_VEL_DIFF = 0.5
TEST = True

class DiffToAck(Node):
    def __init__(self):
        super().__init__("diff_to_ack")
        self.get_logger().info("Converter node has been started")
        self.fwd_vel_diff = 0.0
        self.ang_vel_diff = 0.0
        self.diff_cmd_sub = self.create_subscription(Twist, 'diff_cmd_vel', self.diff_cmd_callback, 10)
        self.ack_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        if TEST:
            self.diff_cmd_pub = self.create_publisher(Twist, 'diff_cmd_vel', 10)
            self.timer = self.create_timer(1.0 / PUBLISH_FREQ, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = FWD_VEL_DIFF
        msg.angular.z = ANG_VEL_DIFF
        self.diff_cmd_pub.publish(msg)
        self.get_logger().info(f"Published forward velocity: {msg.linear.x}, angular velocity: {msg.angular.z}")

    def diff_cmd_callback(self, msg):
        self.fwd_vel_diff = msg.linear.x
        self.ang_vel_diff = msg.angular.z
        self.get_logger().info(f"Received forward velocity: {msg.linear.x}, angular velocity: {msg.angular.z}")
        self.conversion_callback()

    def conversion_callback(self):
        msg = Twist()
        
        # assuming self.fwd_vel_diff is positive
        if np.abs(self.fwd_vel_diff) > FWD_VEL_LOW:
            msg.linear.x = self.fwd_vel_diff
            msg.angular.z = np.arctan(self.ang_vel_diff*WHEEL_BASE/self.fwd_vel_diff)
            #print("nominal case")
        else:
            if np.abs(self.ang_vel_diff) < ANG_VEL_LOW:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                #print("not moving case")
            else:
                msg.angular.z = np.sign(self.ang_vel_diff)*np.pi/2
                msg.linear.x = self.ang_vel_diff/(np.tan(DELTA_MAX)/WHEEL_BASE)
                #print("spinning on the spot case")

        self.ack_cmd_pub.publish(msg)
        self.get_logger().info(f"Published forward velocity: {msg.linear.x}, steer angle: {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    converter_node = DiffToAck()
    rclpy.spin(converter_node)
    converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
