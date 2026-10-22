"""Display DiffBot IMU data in a readable terminal format."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuMonitor(Node):
    """Subscribe to IMU data and periodically print its useful values."""

    def __init__(self):
        """Create the IMU subscriber and the terminal update timer."""
        super().__init__('imu_monitor')

        self.declare_parameter('imu_topic', '/diffbot/imu/data')
        self.declare_parameter('display_rate', 2.0)

        imu_topic = self.get_parameter('imu_topic').value
        display_rate = self.get_parameter('display_rate').value
        if display_rate <= 0.0:
            raise ValueError('display_rate must be greater than zero')

        # The callback stores each new sensor_msgs/msg/Imu message.
        self.latest_imu = None
        self.imu_subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10,
        )

        # A timer controls terminal output, so we do not print all 100 IMU
        # messages per second.
        self.display_timer = self.create_timer(
            1.0 / display_rate,
            self.display_imu,
        )
        self.get_logger().info(f'Monitoring IMU topic: {imu_topic}')

    def imu_callback(self, message: Imu):
        """Store the most recent IMU message received from the sensor."""
        self.latest_imu = message

    def display_imu(self):
        """Convert orientation to degrees and print the newest IMU reading."""
        if self.latest_imu is None:
            self.get_logger().info('Waiting for IMU data...')
            return

        orientation = self.latest_imu.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        angular = self.latest_imu.angular_velocity
        acceleration = self.latest_imu.linear_acceleration

        self.get_logger().info(
            'Orientation [deg]  '
            f'roll={math.degrees(roll):6.1f} '
            f'pitch={math.degrees(pitch):6.1f} '
            f'yaw={math.degrees(yaw):6.1f} | '
            'Angular [rad/s] '
            f'z={angular.z:6.3f} | '
            'Acceleration [m/s^2] '
            f'x={acceleration.x:6.3f} '
            f'y={acceleration.y:6.3f} '
            f'z={acceleration.z:6.3f}'
        )

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert a ROS quaternion into roll, pitch, and yaw in radians."""
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return roll, pitch, yaw


def main(args=None):
    """Start the IMU monitor node."""
    rclpy.init(args=args)
    node = ImuMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
