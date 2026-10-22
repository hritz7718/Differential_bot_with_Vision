"""Run YOLO on DiffBot camera images and publish 2D detections."""

import os

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


class YoloDetector(Node):
    """Detect configured object classes in the RGB camera stream."""

    def __init__(self):
        """Load YOLO and create the ROS image and detection interfaces."""
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/diffbot/camera/image_raw')
        self.declare_parameter('annotated_image_topic','/diffbot/yolo/annotated_image',)
        self.declare_parameter('detections_topic', '/diffbot/yolo/detections')
        package_share = get_package_share_directory('my_diffbot_vision')
        default_model = os.path.join(package_share, 'models', 'yolo11n.pt')
        self.declare_parameter('model', default_model)
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('follow_classes', ['bottle', 'cell phone', 'chair',])
        self.declare_parameter('avoid_classes', ['person'])
        self.declare_parameter('annotate_all', True)

        image_topic = self.get_parameter('image_topic').value
        annotated_image_topic = self.get_parameter('annotated_image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        model_name = self.get_parameter('model').value

        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').value
        self.follow_classes = set(self.get_parameter('follow_classes').value)
        self.avoid_classes = set(self.get_parameter('avoid_classes').value)
        self.annotate_all = self.get_parameter('annotate_all').value

        if self.process_every_n_frames < 1:
            raise ValueError('process_every_n_frames must be at least one')

        self.bridge = CvBridge()
        self.frame_count = 0

        self.get_logger().info(f'Loading YOLO model: {model_name}')
        # On first use, Ultralytics downloads the model weights if needed.
        self.model = YOLO(model_name)

        self.annotated_image_publisher = self.create_publisher(Image,annotated_image_topic,10,)
        self.detections_publisher = self.create_publisher(Detection2DArray,detections_topic,10,)
        self.image_subscription = self.create_subscription(Image,image_topic,self.image_callback,qos_profile_sensor_data,)

        classes = ', '.join(sorted(self.follow_classes))
        self.get_logger().info(f'YOLO ready: {image_topic}; detecting: {classes}')

    def image_callback(self, image_message):
        """Run inference periodically and publish image-space detections."""
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(image_message,desired_encoding='bgr8',)

        except CvBridgeError as error:
            self.get_logger().error(f'Cannot convert camera image: {error}')
            return

        # verbose=False prevents Ultralytics from printing every frame.
        result = self.model(image,conf=self.confidence_threshold,verbose=False,)[0]
        annotated_image = image.copy()
        detections = Detection2DArray()
        detections.header = image_message.header

        for index, box in enumerate(result.boxes):
            class_id = int(box.cls[0].item())
            class_name = result.names[class_id]
            confidence = float(box.conf[0].item())
            if class_name in self.follow_classes:
                colour = (0, 255, 0)      # Green: follow target
            elif class_name in self.avoid_classes:
                colour = (0, 0, 255)      # Red: avoid
            else:
                colour = (255, 0, 0)      # Blue: detected, but ignored

            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
            self.add_detection(detections,image_message,class_name,confidence,index,
                x_min, y_min, x_max, y_max,)
            
            self.draw_detection(annotated_image,class_name,confidence,
                x_min, y_min, x_max, y_max,)

        self.detections_publisher.publish(detections)
        annotated_message = self.bridge.cv2_to_imgmsg(annotated_image,encoding='bgr8',)
        annotated_message.header = image_message.header
        self.annotated_image_publisher.publish(annotated_message)

    @staticmethod
    def add_detection( detections, image_message, class_name, confidence, index,
            x_min, y_min, x_max, y_max):
        
        """Add one YOLO bounding box to a standard Detection2DArray."""
        detection = Detection2D()
        detection.header = image_message.header
        detection.id = f'{class_name}_{index}'
        detection.bbox.center.position.x = (x_min + x_max) / 2.0
        detection.bbox.center.position.y = (y_min + y_max) / 2.0
        detection.bbox.size_x = x_max - x_min
        detection.bbox.size_y = y_max - y_min

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_name
        hypothesis.hypothesis.score = confidence
        detection.results.append(hypothesis)
        detections.detections.append(detection)

    @staticmethod
    def draw_detection(image,class_name,confidence,
            x_min,y_min,x_max,y_max,colour):
        
        """Draw a green bounding box and class label onto an OpenCV image."""
        top_left = (int(x_min), int(y_min))
        bottom_right = (int(x_max), int(y_max))
        label = f'{class_name}: {confidence:.2f}'

        cv2.rectangle(image, top_left, bottom_right, colour, 2)

        cv2.putText(image,label,(top_left[0], max(20, top_left[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,0.6,colour,2,)


def main(args=None):
    """Start the YOLO detector node."""
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
