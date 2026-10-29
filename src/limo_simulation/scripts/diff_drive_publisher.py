#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class DiffDrivePublisher(Node):
    def __init__(self):
        super().__init__('diff_drive_publisher')

        # robot to drive, e.g. robot_0 (empty: use the node namespace)
        robot = self.declare_parameter('robot', '').value
        topic = 'diff_drive_base_controller/cmd_vel'
        if robot:
            topic = f'/{robot}/{topic}'
        self.publisher_ = self.create_publisher(TwistStamped, topic, 10)

        self.timer = self.create_timer(0.03, self.publish_command)  # 10ms = 100Hz
        self.get_logger().info(f'DiffDrivePublisher started on {self.publisher_.topic_name}.')

    def publish_command(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = -1.0
        msg.twist.angular.z = 2.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
