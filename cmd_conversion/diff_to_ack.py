import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np


# ---- Constants ----
WHEEL_BASE = 0.550 # (m) Wheelbase of the Ackermann vehicle. 'L' in the bicycle model.
FWD_VEL_LOW = 1e-3 # (m/s) Threshold for considering forward velocity to be effectively zero
ANG_VEL_LOW = 1e-6 # (rad/s) Threshold for considering angular velocity to be effectively zero 
DELTA_MAX = 0.69   # (rad) Maximum steering angle for the Ackermann vehicle in radians
PUBLISH_FREQ = 1   # (Hz) Frequency for the dummy differential drive command 
FWD_VEL_DIFF = 0.5 # (m/s) dummy differential drive command for forward velocity
ANG_VEL_DIFF = 0.5 # (rad/s) dummy differential drive command for angular velocity
TEST = True        # Flag to activate/deactivate dummy differential command publisher

# ---- Classes ---- 
class DiffToAck(Node):
    """
    A ROS 2 node that converts Twist commands for a differential drive robot
    to Twist commands suitable for an Ackermann steering robot (bicycle model).

    Subscribes to:
      'diff_cmd_vel' (geometry_msgs/Twist): Velocity commands for differential drive.
                                            linear.x is forward velocity, angular.z is angular velocity.
    Publishes:
      'cmd_vel' (geometry_msgs/Twist): Velocity commands for Ackermann drive.
                                       linear.x is forward velocity, angular.z is steering angle.
    If TEST is True, it also publishes dummy commands to 'diff_cmd_vel'.
    """
    def __init__(self):
        super().__init__("diff_to_ack")
        self.get_logger().info("Converter node has been started")
        
        # Initialize instance variables for storing received differential drive commands
        self.fwd_vel_diff = 0.0
        self.ang_vel_diff = 0.0
        
        # Subscriber to the differential drive commands
        self.diff_cmd_sub = self.create_subscription(Twist, 'diff_cmd_vel', self.diff_cmd_callback, 10)
        
        # Publisher for the Ackermann drive commands
        self.ack_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # If TEST mode is enabled, set up a dummy publisher and a timer to send test commands
        if TEST:
            self.diff_cmd_pub = self.create_publisher(Twist, 'diff_cmd_vel', 10)
            self.timer = self.create_timer(1.0 / PUBLISH_FREQ, self.timer_callback)

    def timer_callback(self):
        """
        Timer callback that publishes dummy differential drive commands if TEST is True
        """
        msg = Twist()
        msg.linear.x = FWD_VEL_DIFF
        msg.angular.z = ANG_VEL_DIFF
        self.diff_cmd_pub.publish(msg)
        self.get_logger().info(f"Published forward velocity: {msg.linear.x}, angular velocity: {msg.angular.z}")

    def diff_cmd_callback(self, msg):
        """
        Callback function for processing incoming differential drive Twist commands
        Stores the received velocities and triggers the conversion

        Args:
            msg (geometry_msgs.msg.Twist): The incoming differential drive command
        """
        self.fwd_vel_diff = msg.linear.x
        self.ang_vel_diff = msg.angular.z
        self.get_logger().info(f"Received forward velocity: {msg.linear.x}, angular velocity: {msg.angular.z}")
        self.conversion_callback()

    def conversion_callback(self):
        """
        Performs the conversion from differential drive commands to Ackermann commands and publishes the result 
        
        Handles different cases based on input velocities
        """
        msg = Twist()
        
        # Assuming self.fwd_vel_diff is positive
        if np.abs(self.fwd_vel_diff) > FWD_VEL_LOW:
            # "Nominal" driving: significant forward velocity.
            # This is the standard bicycle model conversion.
            msg.linear.x = self.fwd_vel_diff
            msg.angular.z = np.arctan(self.ang_vel_diff*WHEEL_BASE/self.fwd_vel_diff)
        else:
            if np.abs(self.ang_vel_diff) < ANG_VEL_LOW:
                # Very low angular velocity, robot is effectively stationary
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                # Set steering angle to maximum possible in the direction of rotation
                # Note: publishing pi/2 might be beyond physical limits for DELTA_MAX
                msg.angular.z = np.sign(self.ang_vel_diff)*np.pi/2
                # This attempts to match the turning radius of the differential drive (which would be close to 0)
                msg.linear.x = self.ang_vel_diff/(np.tan(DELTA_MAX)/WHEEL_BASE)

        self.ack_cmd_pub.publish(msg)
        self.get_logger().info(f"Published forward velocity: {msg.linear.x}, steer angle: {msg.angular.z}")

# ---- Functions ----
def main(args=None):
    """
    Main function to initialize and run the ROS 2 node
    """
    rclpy.init(args=args)
    converter_node = DiffToAck()
    rclpy.spin(converter_node)
    converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
