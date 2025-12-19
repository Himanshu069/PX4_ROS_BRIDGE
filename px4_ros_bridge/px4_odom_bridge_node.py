# px4_to_odom_bridge.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from builtin_interfaces.msg import Time


class PX4ToOdomBridge(Node):
    def __init__(self):
        super().__init__('px4_to_odom_bridge')

        self.declare_parameter('px4_ns', '')
        self.declare_parameter('vehicle_ns', 'x500_drone_0')

        px4_ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        vehicle_ns = self.get_parameter('vehicle_ns').get_parameter_value().string_value
        # Create a custom QoS profile to match PX4's publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match PX4's default
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        px4_topic = (
            f'/{px4_ns}/fmu/out/vehicle_odometry'
            if px4_ns else
            '/fmu/out/vehicle_odometry'
        )

        odom_topic = f'/{vehicle_ns}/odom'
        
        self.subscription = self.create_subscription(
            VehicleOdometry,
            px4_topic,
            self.odom_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.odom_frame = f'{vehicle_ns}/odom'
        self.base_frame = f'{vehicle_ns}/base_link'

        self.get_logger().info(f"Subscribed to: {px4_topic}")
        self.get_logger().info(f"Publishing to:  {odom_topic}")
        self.get_logger().info(f"odom frame:     {self.odom_frame}")
        self.get_logger().info(f"base frame:     {self.base_frame}")

    def odom_callback(self, msg: VehicleOdometry):
        odom_msg = Odometry()

        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position and orientation
        odom_msg.pose.pose.position.x = float(msg.position[0])
        odom_msg.pose.pose.position.y = float(msg.position[1])
        odom_msg.pose.pose.position.z = float(msg.position[2])
        odom_msg.pose.pose.orientation.x = float(msg.q[0])
        odom_msg.pose.pose.orientation.y = float(msg.q[1])
        odom_msg.pose.pose.orientation.z = float(msg.q[2])
        odom_msg.pose.pose.orientation.w = float(msg.q[3])

        # Pose covariance (row-major)
        odom_msg.pose.covariance = [
            float(msg.position_variance[0]), 0, 0, 0, 0, 0,
            0, float(msg.position_variance[1]), 0, 0, 0, 0,
            0, 0, float(msg.position_variance[2]), 0, 0, 0,
            0, 0, 0, float(msg.orientation_variance[0]), 0, 0,
            0, 0, 0, 0, float(msg.orientation_variance[1]), 0,
            0, 0, 0, 0, 0, float(msg.orientation_variance[2])
        ]

        # Linear and angular velocity
        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])

        odom_msg.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(msg.angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(msg.angular_velocity[2])

        # Velocity covariance (same as above)
        odom_msg.twist.covariance = [
            float(msg.velocity_variance[0]), 0, 0, 0, 0, 0,
            0, float(msg.velocity_variance[1]), 0, 0, 0, 0,
            0, 0, float(msg.velocity_variance[2]), 0, 0, 0,
            0, 0, 0, 1e-6, 0, 0,
            0, 0, 0, 0, 1e-6, 0,
            0, 0, 0, 0, 0, 1e-6
        ]

        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4ToOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


