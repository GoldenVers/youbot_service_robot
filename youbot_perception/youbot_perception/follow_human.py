#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolomsgs.msg import HumanPositionArray


class FollowHuman(Node):
    def __init__(self):
        super().__init__('follow_human')

        self.subscription = self.create_subscription(
            HumanPositionArray,
            '/human_positions',
            self.human_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/youbot/cmd_vel', 10)

        self.target_area = 0.15  
        self.center_x = 320.0    
        self.linear_k = 0.3
        self.angular_k = 0.001  # Much smaller for stable heading correction
        self.stop_threshold = 0.03
        self.heading_tolerance = 30  # Degrees tolerance before applying angular correction 

        self.get_logger().info("🚶‍♂️ Follow Human Node started")

    def human_callback(self, msg: HumanPositionArray):
        twist = Twist()

        if not msg.humans:
            # No human detected => stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("⛔ No human detected — stopping")
            return

        # Track the largest human (closest/most prominent)
        target = max(msg.humans, key=lambda h: h.width * h.height)
        area = (target.width * target.height) / (640 * 480)
        error_x = target.x_center - self.center_x

        # Control distance: move forward/backward to reach target area
        diff = self.target_area - area
        if abs(diff) > self.stop_threshold:
            twist.linear.x = self.linear_k * diff * 100  
        else:
            twist.linear.x = 0.0  # Stop when at target distance

        # Control heading: only apply gentle angular correction
        # Only correct if error is significant (more than heading_tolerance)
        if abs(error_x) > self.heading_tolerance:
            # Very gentle angular correction to avoid oscillation
            twist.angular.z = -self.angular_k * error_x
        else:
            twist.angular.z = 0.0

        # Limit velocities
        twist.linear.x = max(min(twist.linear.x, 0.3), -0.3)
        twist.angular.z = max(min(twist.angular.z, 0.2), -0.2)  # Reduced angular limit

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f"🎯 Tracking human: area={area:.3f}, err_x={error_x:.1f}, v={twist.linear.x:.2f}, w={twist.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowHuman()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()