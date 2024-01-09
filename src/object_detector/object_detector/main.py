import rclpy
from object_detector.object_detector_node import ObjectDetectorNode
from object_detector.yolov3_detector import YOLOv3Detector
from object_detector.object_detector import ObjectDetector, TestDetector

def main(args=None):
    rclpy.init(args=args)
    detector = YOLOv3Detector()
    node = ObjectDetectorNode("object_detector_node", detector)
    rclpy.spin(node)
    rclpy.shutdown()