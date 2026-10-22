"""Follow detected target classes with camera detections and LiDAR safety."""

import math

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray


class TargetFollower(Node):
    """Follow targets while treating people and LiDAR hits as hazards."""

    def __init__(self):
        """Create ROS interfaces, parameters, and the control timer."""
        super().__init__('target_follower')

        self.declare_parameter('detections_topic', '/diffbot/yolo/detections')
        self.declare_parameter('scan_topic', '/diffbot/scan')
        self.declare_parameter('odom_topic', '/diffbot/odom')
        self.declare_parameter('cmd_vel_topic', '/diffbot/cmd_vel')
        self.declare_parameter('follow_classes', ['bottle', 'bowl'])
        self.declare_parameter('avoid_classes', ['person'])
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('camera_horizontal_fov', 1.047)
        self.declare_parameter('association_half_angle', 0.09)
        self.declare_parameter('target_distance', 0.40)
        self.declare_parameter('stop_distance', 0.25)
        self.declare_parameter('person_stop_distance', 0.6)
        self.declare_parameter('max_target_distance', 4.0)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.60)
        self.declare_parameter('search_speed', 0.08)
        self.declare_parameter('search_turn_speed', 0.35)
        self.declare_parameter('side_clearance_distance', 0.45)
        self.declare_parameter('side_sector_degrees', 70.0)
        self.declare_parameter('detection_timeout', 0.75)
        self.declare_parameter('world_merge_distance', 0.40)

        self.follow_classes = set(
            self.get_parameter('follow_classes').value
        )
        self.avoid_classes = set(
            self.get_parameter('avoid_classes').value
        )
        self.latest_detections = None
        self.latest_scan = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False
        self.world_objects = []

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            self.detections_callback,
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/diffbot/target_pose',
            10,
        )
        self.world_objects_publisher = self.create_publisher(
            PoseArray,
            '/diffbot/detected_object_poses',
            10,
        )
        self.control_timer = self.create_timer(0.1, self.control_robot)

        follow = ', '.join(sorted(self.follow_classes))
        avoid = ', '.join(sorted(self.avoid_classes))
        self.get_logger().info(
            f'Following [{follow}], avoiding [{avoid}] with SEARCH/FOLLOW.'
        )

    def detections_callback(self, message):
        """Store the newest camera detections and estimate their odom poses."""
        self.latest_detections = message
        self.update_world_objects(message)

    def scan_callback(self, message):
        """Store the latest LiDAR scan for range and collision checks."""
        self.latest_scan = message

    def odom_callback(self, message):
        """Store the robot odometry pose used to express objects in odom."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.robot_x = position.x
        self.robot_y = position.y
        self.robot_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.have_odom = True

    def control_robot(self):
        """Publish a safe command based on target, person, and LiDAR data."""
        command = Twist()
        if self.latest_scan is None:
            self.cmd_vel_publisher.publish(command)
            return

        person = None
        target = None
        if self.detections_are_fresh():
            person = self.select_detection(self.avoid_classes)
            target = self.select_detection(self.follow_classes)

        if person is not None:
            person_distance = self.distance_for_detection(person)
            if (
                person_distance is not None
                and person_distance
                < self.get_parameter('person_stop_distance').value
            ):
                self.get_logger().warn(
                    'Person in the route; stopping for safety.'
                )
                self.cmd_vel_publisher.publish(command)
                return

        if target is None:
            command = self.search_command()
            self.cmd_vel_publisher.publish(command)
            return

        target_distance = self.distance_for_detection(target)
        if target_distance is None:
            self.cmd_vel_publisher.publish(command)
            return

        if target_distance > self.get_parameter('max_target_distance').value:
            self.cmd_vel_publisher.publish(command)
            return

        bearing = self.bearing_for_detection(target)
        front_clearance = self.front_clearance()
        if (
            front_clearance is None
            or front_clearance < self.get_parameter('stop_distance').value
        ):
            command = self.search_command()
            self.cmd_vel_publisher.publish(command)
            return

        target_distance_goal = self.get_parameter('target_distance').value
        if target_distance > target_distance_goal:
            command.linear.x = min(
                self.get_parameter('forward_speed').value,
                0.4 * (target_distance - target_distance_goal),
            )

        # Positive angular.z turns left. A target left of the image centre has
        # a positive bearing, so this steers the robot towards the target.
        command.angular.z = max(
            -self.get_parameter('turn_speed').value,
            min(
                self.get_parameter('turn_speed').value,
                1.5 * bearing,
            ),
        )
        self.cmd_vel_publisher.publish(command)
        self.publish_target_pose(target_distance, bearing)

    def search_command(self):
        """Explore safely when no target is visible or the route is blocked."""
        command = Twist()
        front = self.front_clearance()
        left = self.sector_clearance(20.0, self.get_parameter(
            'side_sector_degrees'
        ).value)
        right = self.sector_clearance(-self.get_parameter(
            'side_sector_degrees'
        ).value, -20.0)
        stop_distance = self.get_parameter('stop_distance').value
        side_clearance = self.get_parameter('side_clearance_distance').value
        turn_speed = self.get_parameter('search_turn_speed').value

        # Drive forward only when the route ahead is clear. Otherwise, rotate
        # towards the side with the greater observed free space.
        if front is not None and front >= side_clearance:
            command.linear.x = self.get_parameter('search_speed').value
            if left is not None and right is not None:
                command.angular.z = 0.15 if left > right else -0.15
        elif left is None and right is None:
            self.get_logger().warn('No safe LiDAR direction; stopping.')
        elif left is None:
            command.angular.z = -turn_speed
        elif right is None:
            command.angular.z = turn_speed
        elif left >= right and left > stop_distance:
            command.angular.z = turn_speed
        elif right > stop_distance:
            command.angular.z = -turn_speed
        else:
            self.get_logger().warn('Boxed in by obstacles; stopping.')
        return command

    def update_world_objects(self, detections):
        """Maintain an in-memory list of detected object positions in odom."""
        if self.latest_scan is None or not self.have_odom:
            return

        for detection in detections.detections:
            class_name, confidence = self.class_and_confidence(detection)
            if confidence < self.get_parameter('confidence_threshold').value:
                continue

            distance = self.distance_for_detection(detection)
            if distance is None:
                continue

            bearing = self.bearing_for_detection(detection)
            world_x = self.robot_x + distance * math.cos(
                self.robot_yaw + bearing
            )
            world_y = self.robot_y + distance * math.sin(
                self.robot_yaw + bearing
            )
            self.merge_world_object(class_name, world_x, world_y)

        self.publish_world_objects(detections.header)

    def merge_world_object(self, class_name, world_x, world_y):
        """Update a nearby object of the same class or record a new one."""
        merge_distance = self.get_parameter('world_merge_distance').value
        for object_pose in self.world_objects:
            is_same_class = object_pose['class_name'] == class_name
            distance = math.hypot(
                object_pose['x'] - world_x,
                object_pose['y'] - world_y,
            )
            if is_same_class and distance < merge_distance:
                object_pose['x'] = (object_pose['x'] + world_x) / 2.0
                object_pose['y'] = (object_pose['y'] + world_y) / 2.0
                return

        self.world_objects.append({
            'class_name': class_name,
            'x': world_x,
            'y': world_y,
        })

    def publish_world_objects(self, header):
        """Publish remembered object positions in the odom coordinate frame."""
        message = PoseArray()
        message.header = header
        message.header.frame_id = 'odom'
        for object_pose in self.world_objects:
            pose = PoseStamped().pose
            pose.position.x = object_pose['x']
            pose.position.y = object_pose['y']
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self.world_objects_publisher.publish(message)

    def publish_target_pose(self, distance, bearing):
        """Publish the currently followed target position in odom."""
        if not self.have_odom:
            return

        message = PoseStamped()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self.robot_x + distance * math.cos(
            self.robot_yaw + bearing
        )
        message.pose.position.y = self.robot_y + distance * math.sin(
            self.robot_yaw + bearing
        )
        message.pose.orientation.w = 1.0
        self.target_pose_publisher.publish(message)

    def select_detection(self, class_names):
        """Return the largest confident detection for the given classes."""
        candidates = []
        for detection in self.latest_detections.detections:
            class_name, confidence = self.class_and_confidence(detection)
            if (
                class_name in class_names
                and confidence
                >= self.get_parameter('confidence_threshold').value
            ):
                area = detection.bbox.size_x * detection.bbox.size_y
                candidates.append((area, detection))
        return max(candidates, default=(None, None))[1]

    @staticmethod
    def class_and_confidence(detection):
        """Extract the most likely YOLO class and its confidence score."""
        if not detection.results:
            return '', 0.0
        hypothesis = detection.results[0].hypothesis
        return hypothesis.class_id, hypothesis.score

    def bearing_for_detection(self, detection):
        """Convert the bounding-box centre from pixels to a camera bearing."""
        image_width = self.get_parameter('image_width').value
        horizontal_fov = self.get_parameter('camera_horizontal_fov').value
        pixel_offset = detection.bbox.center.position.x - image_width / 2.0
        return -pixel_offset / image_width * horizontal_fov

    def distance_for_detection(self, detection):
        """Find the nearest valid LiDAR distance at the detection bearing."""
        bearing = self.bearing_for_detection(detection)
        return self.scan_distance_at_angle(bearing)

    def scan_distance_at_angle(self, target_angle):
        """Return the nearest valid scan range around a requested angle."""
        half_angle = self.get_parameter('association_half_angle').value
        ranges = []
        for index, distance in enumerate(self.latest_scan.ranges):
            if not self.valid_range(distance):
                continue
            angle = self.latest_scan.angle_min + (
                index * self.latest_scan.angle_increment
            )
            angle_difference = math.atan2(
                math.sin(angle - target_angle),
                math.cos(angle - target_angle),
            )
            if abs(angle_difference) <= half_angle:
                ranges.append(distance)
        return min(ranges) if ranges else None

    def front_clearance(self):
        """Return the nearest valid LiDAR range in a 20-degree front sector."""
        ranges = []
        for index, distance in enumerate(self.latest_scan.ranges):
            if not self.valid_range(distance):
                continue
            angle = self.latest_scan.angle_min + (
                index * self.latest_scan.angle_increment
            )
            if abs(angle) <= math.radians(20.0):
                ranges.append(distance)
        return min(ranges) if ranges else None

    def sector_clearance(self, min_degrees, max_degrees):
        """Return the nearest valid range inside an angular LiDAR sector."""
        min_angle = math.radians(min_degrees)
        max_angle = math.radians(max_degrees)
        ranges = []
        for index, distance in enumerate(self.latest_scan.ranges):
            if not self.valid_range(distance):
                continue
            angle = self.latest_scan.angle_min + (
                index * self.latest_scan.angle_increment
            )
            if min_angle <= angle <= max_angle:
                ranges.append(distance)
        return min(ranges) if ranges else None

    def valid_range(self, distance):
        """Check that one LiDAR range is finite and within sensor limits."""
        return (
            math.isfinite(distance)
            and self.latest_scan.range_min <= distance
            <= self.latest_scan.range_max
        )

    def detections_are_fresh(self):
        """Reject old detections so stale camera data cannot move the robot."""
        if self.latest_detections is None:
            return False
        stamp = self.latest_detections.header.stamp
        detection_time = stamp.sec + stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        return 0.0 <= now - detection_time <= self.get_parameter(
            'detection_timeout'
        ).value

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert a quaternion to yaw in radians."""
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )


def main(args=None):
    """Start the target follower node."""
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
