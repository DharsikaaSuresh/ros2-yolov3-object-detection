import numpy as np

class ObjectDetector:
    """Base class for object detectors."""
    def __init__(self) -> None:
        # The base class should not be instantiated directly; subclasses should implement their own constructor.
        raise NotImplementedError

    def detect(self, image: np.ndarray ):
        """Detect objects in the image and return bounding boxes."""
        # Subclasses must implement the detection logic
        raise NotImplementedError
    

import random
class TestDetector(ObjectDetector):
    """Test detector that returns dummy bounding boxes."""
    def __init__(self) -> None:
        print("TestDetector initialized")

        # Unit bounding boxes representing objects in the image
        unit_boxes = [[[55, 57], [69, 57], [69, 91], [55, 91]],
                        [[56, 27], [91, 27], [91, 96], [56, 96]],
                        [[82, 98], [91, 98], [91, 100], [82, 100]],
                        [[9, 92], [42, 92], [42, 95], [9, 95]],
                        [[27, 32], [75, 32], [75, 66], [27, 66]],
                        [[37, 19], [97, 19], [97, 58], [37, 58]],
                        [[58, 26], [60, 26], [60, 38], [58, 38]],
                        [[88, 30], [95, 30], [95, 93], [88, 93]],
                        [[38, 76], [54, 76], [54, 98], [38, 98]],
                        [[25, 15], [67, 15], [67, 65], [25, 65]]]
        scale_factor = 10

        # Generate sample bounding boxes by scaling the unit boxes
        self.sample_boxes = []
        for box in unit_boxes:
            top_left = box[0]
            top_right = [box[1][0] + (box[1][0] - box[0][0]) * (scale_factor - 1), box[1][1]]
            bottom_right = [box[2][0] + (box[2][0] - box[0][0]) * (scale_factor - 1), 
                            box[2][1] + (box[2][1] - box[0][1]) * (scale_factor - 1)]
            bottom_left = [box[3][0], box[3][1] + (box[3][1] - box[0][1]) * (scale_factor - 1)]
            self.sample_boxes.append([top_left, top_right, bottom_right, bottom_left])


    def detect(self, image: np.ndarray) :
        """Implementation of the detect method for the TestDetector"""
        print("TestDetector detect called")
        dummy_boxes = [] # [top-right, bottom-right, , top-left, bottom-left]  # top-right = [x, y]
        for i in range( random.randint(0, 5)):
            box = self.sample_boxes[random.randint(0, len(self.sample_boxes)-1)]
            dummy_boxes.append(box) 
        return dummy_boxes