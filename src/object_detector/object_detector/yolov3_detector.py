import os
import cv2 as cv
import numpy as np

from ament_index_python.packages import get_package_share_directory
from object_detector.object_detector import ObjectDetector

class YOLOv3Detector(ObjectDetector):
#class YOLOv3Detector():
    def __init__(self):
        # Load YOLOv3 model
        package_share_directory = get_package_share_directory('object_detector')
        yolov3_cfg_path = os.path.join(package_share_directory, 'yolov3_data', 'yolov3.cfg')
        yolov3_weights_path = os.path.join(package_share_directory, 'yolov3_data', 'yolov3.weights')
        coco_names_path = os.path.join(package_share_directory, 'yolov3_data', 'coco.names')
        
        # Load configuration and weights from YOLOv3 model
        self.net = cv.dnn.readNetFromDarknet(yolov3_cfg_path, yolov3_weights_path)
        self.net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        self.ln = [self.net.getLayerNames()[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # Load names of classes
        self.classes = open(coco_names_path).read().strip().split('\n')
        

    def detect(self, img: np.ndarray) -> list:
        # Image preprocessing
        blob = cv.dnn.blobFromImage(img, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.ln)

        # Post-processing
        boxes, confidences, classIDs = self.post_process(img, outputs, 0.5)
        return boxes

    def post_process(self, img, outputs, conf_threshold):
        """Extract bounding boxes, confidences, and class IDs from YOLOv3 outputs"""
        H, W = img.shape[:2]
        boxes = []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                if confidence > conf_threshold:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # Convert to [top-left, top-right, bottom-right, bottom-left]
                    top_left = [x, y]
                    top_right = [x + width, y]
                    bottom_right = [x + width, y + height]
                    bottom_left = [x, y + height]
                    boxes.append([top_left, top_right, bottom_right, bottom_left])
        return boxes, None, None
    

if __name__ == "__main__":
    """Test case to check YOLOv3 functionality"""
    detector = YOLOv3Detector()

    # Read an example image for detection
    img = cv.imread(f'{os.path.dirname(os.path.abspath(__file__))}/yolov3_data/horse.jpg')
    out = detector.detect(img)
    print(out)
