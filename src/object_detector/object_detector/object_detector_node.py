from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

from visualization_msgs.msg import ImageMarker
from foxglove_msgs.msg import ImageMarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from object_detector.object_detector import ObjectDetector


class ObjectDetectorNode(Node):
    def __init__(self, name: str, detector: ObjectDetector) -> None:
        super().__init__(name)

        # Initialize the node with a given name and an ObjectDetector instance
        self.detector = detector
        self.threshold = 10000.0
        self.image_topic = "/d455_1_rgb_image"
        self.camera_info_topic = "/d455_1_rgb_camera_info"

        # Create subscriptions and publisher
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(Image, self.camera_info_topic, self.camera_info_callback, 10)
        self.marker_pub = self.create_publisher(ImageMarkerArray,"/bounding_box_markers", 10)

        # Initialize CvBridge for converting between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
        """Callback function for processing incoming image messages"""
        image = self.bridge.imgmsg_to_cv2(msg)
        self.process_image(image, msg.header)

    def camera_info_callback(self, msg: Image) -> None:
        """Placeholder for camera info callback"""
        pass
    
    def process_image(self, image: Image, header) -> None:
        """Detect object and publish markers"""
        bounding_boxes = self.detector.detect(image)
        bounding_boxes = self.bounding_box_threshold(bounding_boxes, self.threshold)
        markers = self.create_image_marker_from_bounding_box(bounding_boxes, headers=header)
        self.marker_pub.publish(markers)

    def create_image_marker_from_bounding_box(self, bounding_boxes, headers):
        markers = ImageMarkerArray()
        for bounding_box in bounding_boxes:
            # Add points in the order: top-left, top-right, bottom-right, bottom-left
            markers.markers.append(
                ImageMarker(
                    header=headers,
                    type=ImageMarker.POLYGON,
                    outline_color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                    scale=1.0,
                    points=[
                        Point(x=float(bounding_box[0][0]), y=float(bounding_box[0][1]), z=0.0),  # top-left
                        Point(x=float(bounding_box[1][0]), y=float(bounding_box[1][1]), z=0.0),  # top-right
                        Point(x=float(bounding_box[2][0]), y=float(bounding_box[2][1]), z=0.0),  # bottom-right
                        Point(x=float(bounding_box[3][0]), y=float(bounding_box[3][1]), z=0.0)   # bottom-left
                    ],
                )
            )     
        return markers
    
    def bounding_box_threshold(self, bounding_boxes: list, threshold: float) -> list:
        """Filter bounding boxes by area."""
        filtered_boxes = []
        for box in bounding_boxes:
            area = self.calculate_area(box)
            if area > threshold:
                filtered_boxes.append(box)
            else:
                print("Area too small, discarding bounding box")
        return filtered_boxes
    
    def calculate_area(self, box: list) -> float:
        """Calculate area of a bounding box."""
        return (box[1][0] - box[0][0]) * (box[2][1] - box[1][1])
