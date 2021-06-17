# TIAGo_find_object
### A brief introduction

See a blog [here](https://aakaashradhoe.medium.com/3d-object-detection-for-tiago-robot-using-a-faster-r-cnn-network-c2d1768f7490) and there is a recorded video [here](https://drive.google.com/file/d/1VqTg8dFCKsVXLAXaDL9LmUPcT7icDjg0/view?usp=sharing) to show how it works.

The perception part solves a 3D object detection task. In our case for TIAGo, it is designed for recognizing 5 objects in the gazebo simulation: '*biscuits*', '*cocacola*', '*pringles*', '*pringles2*', '*sprite*', with encoded ids from 0 to 4. We trained a Faster R-CNN on our customized dataset(with COCO format) and utilized it for 2D object detection. Each target's 2D bounding box is able to generate a 3D viewing frustum, by which we could filter out the object pointclouds from all possible candidates. Finally, the filtered "clean" pointclouds is inputed to PCL cylinder detector to calculate a 3D pose. We also list the packages and several main topics here:

#### packages

- **find_object**: the main process. It waits for the action call from set_nav_goal node, gets the target info(what object the customer wants now), finds the object, and returns the found object 3D pose back
- **mmdetection_ros**: for deploying MMDetection in ros
- **vision_msgs**: defines messages for object detection

#### custom topics

- `/to_perception`: action topic between set_nav_goal and find_object
- `/mmdetector/objects`: mmdetector publishes the object 2D bounding boxes info
- `/to_cylinder_detector`: find_object publishes the filtered pointclouds
- `/cylinder_detector/cylinder_pose`: pcl cylinder detector publishes the calculated 3D object pose

### Requirements

Besides packages provided by TIAGo, for several requirements are: Python 3.6+, PyTorch 1.3+, CUDA 9.2+ and [MMCV](https://mmcv.readthedocs.io/en/latest/#installation). Don't worry! Let's do it together.

### Installation

1. We use a popular object detection toolbox [MMDetection](https://github.com/open-mmlab/mmdetection), which is a part of OpenMMLab. For using it in ros, I wrote a small package [here](https://github.com/jianfengc11/mmdetection_ros). Follow the installation in README there step by step(ideally it won't take 10 min).  Note that we also use a repo for object detection messages.

2. Now you have all the materials for our perception. Don't forget to compile them.

   ```shell
   catkin build find_object mmdetection_ros vision_msgs
   ```

Now we are done! Let's run the TIAGo.

### Run

1. Run the simulation, detector and the demo scenario:

   ```shell
   # Open a new temnial, start the simulation, and wait for seconds..
   roslaunch YOUR_TIAGO_SIMULATION_PKG YOUR_TIAGO_SIMULATION_LAUNCH_FILE
   # Open a new temnial, start the detector, and wait for seconds..
   source ~/mmdet/bin/activate # in the python3 virtual env
   roslaunch find_object mmdetector.launch
   # Start the demo
   roslaunch set_nav_goal demo.launch
   ```

   Note that after starting the `mmdetector.launch`, Faster R-CNN continuously do the inference on the camera image, with 1 Hz. This process can stress a burden on your gpu, so you may be patient. We are trying to make it more lightweighted. After starting `demo.launch`, the robot moves to the user-defined location and the perception part starts and at the same time PCL cylinder detector is waiting for the filtered pointclouds. 

   In the video, TIAGo moves in front of the 5 objects on the table, look down, find the target(in this case, biscuits), look at it, calculate the 3D pose and send it back to the action client. But we left the code as visualizing it rather than sending it back.