#!/usr/bin/env python3
"""
Object Detection Node - Computer Vision Pipeline

Real-time object detection using YOLOv8 or Isaac ROS for identifying
target objects in camera frames. Publishes detections as vision_msgs/Detection2DArray
with bounding boxes, class IDs, and confidence scores.

This node bridges perception and manipulation by locating objects
specified in LLM task plans.

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class ObjectDetectionNode(Node):
    """
    ROS 2 node for real-time object detection using YOLOv8.

    Subscribes to camera images and publishes detected objects with
    bounding boxes and confidence scores.
    """

    def __init__(self):
        super().__init__('object_detection_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('target_classes_file', 'config/detection_classes.yaml')
        self.declare_parameter('device', 'cpu')  # 'cpu' or 'cuda'
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_annotated', True)

        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.publish_annotated = self.get_parameter('publish_annotated').value

        # Initialize YOLO model
        if not YOLO_AVAILABLE:
            self.get_logger().error(
                "ultralytics package not installed. "
                "Install with: pip install ultralytics"
            )
            raise ImportError("YOLOv8 not available")

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Loaded YOLO model: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # Load target classes (optional filtering)
        self.target_classes = self.load_target_classes()

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.detect_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/perception/detections',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/perception/status',
            10
        )

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/perception/annotated_image',
                10
            )

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

        self.get_logger().info(
            f"Object detection node initialized "
            f"(conf={self.conf_threshold}, device={self.device})"
        )

    def load_target_classes(self):
        """
        Load target object classes from YAML file.

        Returns:
            list: List of target class names (e.g., ['cup', 'mug', 'bottle'])
                  Returns None if file not found (use all classes)
        """
        classes_file = self.get_parameter('target_classes_file').value

        try:
            import yaml
            with open(classes_file, 'r') as f:
                config = yaml.safe_load(f)
                target_classes = config.get('target_objects', [])
                self.get_logger().info(
                    f"Loaded {len(target_classes)} target classes: {target_classes}"
                )
                return target_classes
        except FileNotFoundError:
            self.get_logger().warn(
                f"Target classes file not found: {classes_file}. "
                "Detecting all COCO classes."
            )
            return None
        except Exception as e:
            self.get_logger().error(f"Error loading target classes: {e}")
            return None

    def detect_callback(self, image_msg: Image):
        """
        Process incoming camera image and detect objects.

        Args:
            image_msg: ROS 2 Image message
        """
        self.frame_count += 1

        try:
            # Convert ROS Image to OpenCV format
            frame = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run YOLO inference
        try:
            results = self.model(
                frame,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        # Parse detections
        detections_msg = Detection2DArray()
        detections_msg.header = image_msg.header

        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes

            for box in boxes:
                # Extract box data
                xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]

                # Filter by target classes if specified
                if self.target_classes and cls_name not in self.target_classes:
                    continue

                # Create Detection2D message
                detection = Detection2D()
                detection.header = image_msg.header

                # Bounding box (center + size format)
                center_x = (xyxy[0] + xyxy[2]) / 2.0
                center_y = (xyxy[1] + xyxy[3]) / 2.0
                size_x = xyxy[2] - xyxy[0]
                size_y = xyxy[3] - xyxy[1]

                detection.bbox.center = Pose2D()
                detection.bbox.center.x = float(center_x)
                detection.bbox.center.y = float(center_y)
                detection.bbox.size_x = float(size_x)
                detection.bbox.size_y = float(size_y)

                # Object hypothesis (class + confidence)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = cls_name  # Use string ID
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)

                detections_msg.detections.append(detection)
                self.detection_count += 1

                self.get_logger().debug(
                    f"Detected {cls_name} with confidence {conf:.2f} "
                    f"at ({center_x:.0f}, {center_y:.0f})"
                )

        # Publish detections
        self.detection_pub.publish(detections_msg)

        # Publish status
        if len(detections_msg.detections) > 0:
            self.publish_status(
                f"DETECTED: {len(detections_msg.detections)} objects"
            )
        else:
            self.publish_status("NO_OBJECTS")

        # Publish annotated image (optional)
        if self.publish_annotated and len(results) > 0:
            annotated_frame = results[0].plot()  # Draw bounding boxes
            try:
                annotated_msg = self.cv_bridge.cv2_to_imgmsg(
                    annotated_frame,
                    encoding='bgr8'
                )
                annotated_msg.header = image_msg.header
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")

        # Log statistics (every 100 frames)
        if self.frame_count % 100 == 0:
            avg_detections = self.detection_count / self.frame_count
            self.get_logger().info(
                f"Processed {self.frame_count} frames, "
                f"avg {avg_detections:.2f} detections/frame"
            )

    def publish_status(self, status: str):
        """
        Publish detection status for integration demo.

        Args:
            status: Status string (e.g., "DETECTED: 2 objects", "NO_OBJECTS")
        """
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    object_detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        object_detection_node.get_logger().info(
            "Object detection node stopped by user"
        )
    finally:
        object_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
