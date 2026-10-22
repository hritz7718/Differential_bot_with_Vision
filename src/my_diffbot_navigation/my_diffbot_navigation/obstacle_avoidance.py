"""A small, readable reactive obstacle-avoidance ROS 2 node."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    """Drive forward when clear and turn toward the more open side."""

    def __init__(self):
        """Create the ROS interfaces and configurable parameters."""
        super().__init__('obstacle_avoidance')

        # Parameters let us tune behaviour at launch time without editing code.
        self.declare_parameter('scan_topic', '/diffbot/scan')
        self.declare_parameter('cmd_vel_topic', '/diffbot/cmd_vel')
        self.declare_parameter('clear_distance', 0.60)
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.60)
        self.declare_parameter('front_sector_degrees', 30.0)

        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # A publisher sends Twist velocity messages to the drive plugin.
        self.cmd_vel_publisher = self.create_publisher(
            Twist, cmd_vel_topic, 2
        )

        # A subscriber runs scan_callback whenever the LiDAR publishes a scan.
        self.scan_subscription = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.get_logger().info(
            f'Obstacle avoidance ready: {scan_topic} -> {cmd_vel_topic}'
        )

    def scan_callback(self, scan: LaserScan):
        """Choose a safe Twist command from LiDAR ranges."""
        front_half_angle = math.radians(
            self.get_parameter('front_sector_degrees').value
        )
        front_ranges = []
        left_ranges = []
        right_ranges = []

        for index, distance in enumerate(scan.ranges):
            # Ignore invalid values such as inf, NaN, and measurements outside
            # the sensor's declared range.
            if (
                not math.isfinite(distance)
                or not scan.range_min <= distance <= scan.range_max
            ):
                continue

            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) <= front_half_angle:
                front_ranges.append(distance)
            elif angle > 0.0:
                left_ranges.append(distance)
            else:
                right_ranges.append(distance)

        command = Twist()
        if not front_ranges:
            # A missing forward measurement is unsafe: do not drive blindly.
            self.get_logger().warn('No valid front LiDAR measurements; stopping.')
        else:
            front_distance = min(front_ranges)
            clear_distance = self.get_parameter('clear_distance').value
            stop_distance = self.get_parameter('stop_distance').value

            if front_distance >= clear_distance:
                command.linear.x = self.get_parameter('forward_speed').value
            else:
                # Turn toward whichever side has more room. Missing side
                # readings are treated as blocked, so the decision is cautious.
                left_clearance = min(left_ranges) if left_ranges else 0.0
                right_clearance = min(right_ranges) if right_ranges else 0.0
                turn_direction = (
                    1.0 if left_clearance >= right_clearance else -1.0
                )
                turn_speed = self.get_parameter('turn_speed').value
                command.angular.z = turn_direction * turn_speed

                # Between stop_distance and clear_distance, continue forward
                # slowly while turning. Inside stop_distance, rotate in place.
                if front_distance > stop_distance:
                    fraction = (front_distance - stop_distance) / (
                        clear_distance - stop_distance
                    )
                    command.linear.x = (
                        fraction * self.get_parameter('forward_speed').value
                    )

        self.cmd_vel_publisher.publish(command)


def main(args=None):
    """Start the ROS 2 node and keep it running until interrupted."""
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a final zero command so simulated or real hardware stops safely.
        node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
