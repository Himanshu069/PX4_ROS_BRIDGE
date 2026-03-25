#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined, VehicleAttitude
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

class PX4IMUBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        pub_qos = QoSProfile(
             reliability=ReliabilityPolicy.RELIABLE,
             durability=DurabilityPolicy.VOLATILE,
             depth=10
        )
        self.declare_parameter('gyro_noise', 0.0150)
        self.declare_parameter('accel_noise', 0.3500)
        

        gyro_noise = self.get_parameter('gyro_noise').value
        accel_noise = self.get_parameter('accel_noise').value
        self.gyro_var = gyro_noise ** 2    # 0.000225
        self.accel_var = accel_noise ** 2 
        
        self.declare_parameter('px4_ns', '')
        self.declare_parameter('vehicle_ns', 'x500_drone_0')

        px4_ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        vehicle_ns = self.get_parameter('vehicle_ns').get_parameter_value().string_value
       
        if px4_ns:
            px4_topic = f'{px4_ns}/fmu/out/sensor_combined'
        else:
            px4_topic = '/fmu/out/sensor_combined'
        imu_topic = f'/{vehicle_ns}/imu/data_raw'
        self.sub = self.create_subscription(SensorCombined,
                                            px4_topic,
                                            self.callback, qos_profile )
        self.pub = self.create_publisher(
            Imu,
            imu_topic,
            pub_qos
        )
        self.attitude_sub = self.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude',
                self.attitude_callback,
                qos_profile
        )
        self.latest_attitude = None
        self.frame_id = f"{vehicle_ns}/imu_sensor"

        self.get_logger().info(f"Subscribed to: {px4_topic}")
        self.get_logger().info(f"Publishing to: {imu_topic}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")

    def attitude_callback(self,msg):
        self.latest_attitude = msg

    def callback(self, msg):
        imu_msg = Imu()
        # Timestamp (approximate)
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        #imu_msg.header.stamp = now
       # stamp_sec = msg.timestamp // 1_000_000
       # stamp_nsec = (msg.timestamp % 1_000_000) * 1000
       # imu_msg.header.stamp.sec = int(stamp_sec)
       # imu_msg.header.stamp.nanosec = int(stamp_nsec)
        imu_msg.header.frame_id = self.frame_id


        imu_msg.angular_velocity.x = float(msg.gyro_rad[1])
        imu_msg.angular_velocity.y = -float(msg.gyro_rad[0])
        imu_msg.angular_velocity.z = -float(msg.gyro_rad[2])

        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.y = -float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.z = -float(msg.accelerometer_m_s2[2])
        
        gv = self.gyro_var
        av = self.accel_var

        imu_msg.angular_velocity_covariance = [
            gv,  0.0, 0.0,
            0.0, gv,  0.0,
            0.0, 0.0, gv
        ]
        imu_msg.linear_acceleration_covariance = [
            av,  0.0, 0.0,
            0.0, av,  0.0,
            0.0, 0.0, av
        ]

        if self.latest_attitude is not None:
            q = self.latest_attitude.q
            w_ned = float(q[0])
            x_ned = float(q[1])
            y_ned = float(q[2])
            z_ned = float(q[3])
            imu_msg.orientation.w =  w_ned
            imu_msg.orientation.x =  x_ned
            imu_msg.orientation.y =  y_ned
            imu_msg.orientation.z =  z_ned           
            ov = 0.05
            imu_msg.orientation_covariance = [
                ov,  0.0, 0.0,
                0.0, ov,  0.0,
                0.0, 0.0, ov
            ]
        else:
            imu_msg.orientation_covariance[0] = -1.0  # marks "no orientation estimate"

        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

