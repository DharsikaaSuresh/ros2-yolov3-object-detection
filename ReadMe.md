# Object Detector


This repository contains a ros2 package for object detection using YOLOv3 and visualization using Foxglove Studio. 


## Installation

1. Install Foxglove Studio:

   ```bash
   sudo snap install foxglove-studio
   ```

2. Install the ros bridge server:

   ```bash
   sudo apt-get install ros-humble-rosbridge-server
    ```

3. Install the message packages:

   ```bash
   sudo apt install ros-humble-vision-msgs
   sudo apt install ros-humble-sensor-msgs
   sudo apt install ros-humble-foxglove-msgs
   ```

4. Clone the repository:

   ```bash
   git clone https://github.com/DharsikaaSuresh/ros2-yolov3-object-detection.git
    ```

5. Download weights using setup script:
    It will download weights for YOLOv3 if not already present.

   ```bash
   bash src/object_detector/setup-env.sh
    ```
6. Build:
    
    ```bash
    colcon build
    ```


## Usage

1. Source the workspace.

    ```bash
    source install/setup.bash
    ```

2. Launch Foxglove Studio and open the project file.

    ```bash
    foxglove-studio
    ```

    config file: `src/object_detector/config/object_detection_viz.json`



3. Launch the nodes with launch file.
    
    ```bash
    ros2 launch object_detector object_detector.launch.py
    ```

4. Play the ROS bag file using the following command.
    ```bash
    ros2 bag play /path/to/your/bagfile.bag
    ```
    Replace `/path/to/your/bagfile.bag` with the actual path of ROS 2 bag file.
