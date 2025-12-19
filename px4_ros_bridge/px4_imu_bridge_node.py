#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PX4IMUBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.declare_parameter('px4_ns', '')
        self.declare_parameter('vehicle_ns', 'x500_drone_0')

        px4_ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        vehicle_ns = self.get_parameter('vehicle_ns').get_parameter_value().string_value
        
        px4_topic = f'/{px4_ns}/fmu/out/sensor_combined'
        imu_topic = f'/{vehicle_ns}/imu/data'

        self.sub = self.create_subscription(SensorCombined,
                                            px4_topic,
                                            self.callback, qos_profile )
        self.pub = self.create_publisher(
            Imu,
            imu_topic,
            qos_profile
        )
        self.frame_id = f"{vehicle_ns}/camera_link/StereoOV7251"

        self.get_logger().info(f"Subscribed to: {px4_topic}")
        self.get_logger().info(f"Publishing to: {imu_topic}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")

    def callback(self, msg):
        imu_msg = Imu()
        # Timestamp (approximate)
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id


        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = float(msg.gyro_rad[1])
        imu_msg.angular_velocity.z = float(msg.gyro_rad[2])

        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = float(msg.accelerometer_m_s2[2])

        # No orientation data in SensorCombined → leave orientation unset (NaNs)
        imu_msg.orientation_covariance[0] = -1  # marks "no orientation estimate"

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

