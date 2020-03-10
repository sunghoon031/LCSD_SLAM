#  **Loosely-Coupled Semi-Direct Monocular SLAM**

Paper: https://arxiv.org/abs/1807.10073

Supplementary Material: [Here](https://github.com/sunghoon031/LCSD_SLAM/blob/master/Supplementary_material.pdf)

Video demo: https://youtu.be/j7WnU7ZpZ8c

## :large_blue_diamond: 1. Raw Results and MATLAB Scripts (for figures).
Run `main.m` after setting appropriate paths and mode:
- Mode 1: Plot pre-evaluated results to make the same figures in the paper.
- Mode 2: Process your own results and plot.
- Mode 3: Plot your processed results (if you've already run with mode=2).

## :large_blue_diamond: 2. C++ Source Code
### 2.1. License
Our source-code is licensed under the [GNU General Public License Version 3 (GPLv3)](https://github.com/sunghoon031/LSV-SLAM/blob/master/LICENSE).

### 2.2. Prerequisites
Our implementation is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and [DSO](https://github.com/JakobEngel/dso).
So please install the required (and recommended) dependencies for each project first.
These are: ROS, C++11 compiler, suitesparse, eigen3, Pangolin, and OpenCV. 
The instructions can be found in their original project pages.
We tested our code using Ubuntu 14.04 (ROS Indigo) and Ubuntu 16.04 (ROS Kinetic).

### 2.3. How to build:

1. Install ROS.
2. Install dependencies:
````
sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev libopencv-dev
````
2. Install [Pangolin](https://github.com/stevenlovegrove/Pangolin).
3. Install [ziplib](https://github.com/JakobEngel/dso#ziplib-recommended) (recommended)
````
sudo apt-get install zlib1g-dev
cd ~/LCSD_SLAM/DSO/thirdparty
tar -zxvf libzip-1.1.1.tar.gz
cd libzip-1.1.1/
./configure
make
sudo make install
sudo cp lib/zipconf.h /usr/local/include/zipconf.h  
````
4. Build!
````
cd [YOUR_PATH]/LCSD_SLAM
chmod +x build.sh

// In build.sh, change line 1 and 40 by replacing "~/LCSD_SLAM" to "[YOUR_PATH]/LCSD_SLAM"
./build.sh
````


### 2.4. How to run: 

#### (1) Enable/Disable visualization (GUI):
&nbsp;&nbsp;&nbsp;&nbsp;(a) GUI for the direct module (DSO): Go to `DSO_ROS/catkin_ws/src/dso_ros/launch/euroc_***.launch` (for the EuRoC MAV dataset) or `DSO_ROS/catkin_ws/src/dso_ros/launch/monoVO_***.launch` (for the TUM monoVO dataset) and set `display_GUI` to either true or false.

&nbsp;&nbsp;&nbsp;&nbsp;(b) GUI for the feature-based module (ORB-SLAM): Go to `ORB_SLAM2/Examples/Monocular/EuRoC_seong_***.yaml` (for the EuRoC MAV dataset) or `ORB_SLAM2/Examples/Monocular/TUMmonoVO_yaml/TUM_monoVO_***.yaml` (for the TUM monoVO dataset) and set `GUI` to either 1 or 0.

Note that enabling both GUIs can slow down the performance. The results reported in the paper were obtained by running the system with both GUIs disabled. If you really want the optimal result with the visualization, reduce the playback speed by half (see next) and enable the GUI of ORB-SLAM only. 

#### (2) Set the playback speed and paths:
This can be done by setting the parameter value of `playback_speed` in the launch file in DSO_ROS (see [Step 1(a)](https://github.com/sunghoon031/LCSD-SLAM/blob/master/README.md#1-enabledisable-visualization-gui)). The default is 1 (original speed).

Also, edit the input paths (`image_file_path`, `calib_file_path`, `vignette_file_path`, `gamma_file_path`) and the output path (`stats_file_path`).

For TUM_monoVO dataset, the four input paths should be directed to `images.zip`, `camera.txt`, `vignette.png`, `pcalib.txt` provided by the [dataset](https://vision.cs.tum.edu/data/datasets/mono-dataset).

For EuRoC dataset, the `image_file_path` should be directed to the folder containg the images, and `calib_file_path` should be directed to either `cam0_calib_for_dso.txt` or `cam1_calib_for_dso.txt` provided [here](cam0_calib_for_dso.txt) and [here](cam1_calib_for_dso.txt). 

#### (3) [For datasets other than TUM_monoVO only, e.g., EuRoC MAV or KITTI] Create times.txt 
You need to provide the timestamp data separately. Create `times.txt` and write the image timestamps in one column (the unit doesn't matter). Make sure you have the same number of timestamps as the number of images in `image_file_path`. Then move this file to the same directory where your `image_file_path` is located. 

Example with EuRoC_MAV dataset:
````
MH_01_easy
|
└─── cam0
     |    times.txt
     |
     └─── data  #This is your image_file_path
          |   img0.png
          |   img1.png
          |   ...
````

#### (4) Run:
&nbsp;&nbsp;&nbsp;&nbsp;(i) In the first terminal, run `roscore`

&nbsp;&nbsp;&nbsp;&nbsp;(ii) In the second terminal, run
````
cd [YOUR_PATH]/LCSD_SLAM/ORB_SLAM2

// For the EuRoC MAV dataset:
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt \
Examples/Monocular/EuRoC_seong_[VO/SLAM]_cam[0/1].yaml 

// For the TUM monoVO dataset:
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt \
Examples/Monocular/TUMmonoVO_yaml/monoVO_ORB_[VO/SLAM]_full_[SEQUENCE_NUMBER].yaml 
````
&nbsp;&nbsp;&nbsp;&nbsp;(iii) In the third terminal, run
````
cd [YOUR_PATH]/DSO_ROS/catkin_ws
source devel/setup.bash

// For the EuRoC MAV dataset (see launch files in LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros/launch):
roslaunch dso_ros EuRoC_seong_[SEQUENCE_NAME]_cam[0/1].launch 

// For the TUM monoVO dataset (see launch files in LCSD_SLAM/DSO_ROS/catkin_ws/src/dso_ros/launch):
roslaunch dso_ros monoVO_seong_[SEQUENCE_NUMBER].launch
````
The keyframe trajectory will be saved as `KeyFrameTrajectory_seong.txt` in `ORB_SLAM2` folder, and additional tracking statistics will be saved as `trackingStats.txt` in the path you set (`stats_file_path`) in [Step 3](https://github.com/sunghoon031/LCSD-SLAM/blob/master/README.md#3-set-paths).

### 2.4. Important source files: 
1. [ros_output_wraper.cpp](https://github.com/sunghoon031/LCSD_SLAM/blob/master/DSO_ROS/catkin_ws/src/dso_ros/src/ros_output_wrapper.cpp): Collects the marginalized data from DSO

2. [dso_node.cpp](https://github.com/sunghoon031/LCSD_SLAM/blob/master/DSO_ROS/catkin_ws/src/dso_ros/src/dso_node.cpp): Runs DSO and sends the marginalized data to ORB-SLAM.

3. [ros_mono.cc](https://github.com/sunghoon031/LCSD_SLAM/blob/master/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/ros_mono.cc): Runs ORB-SLAM and receives the marginalized data from DSO.

4. [Tracking.cc](https://github.com/sunghoon031/LCSD_SLAM/blob/master/ORB_SLAM2/src/Tracking.cc): Performs motion-only geometric BA using the initial pose estimate from DSO. Also, it decides whether or not the new keyframe should be inserted (and more points from DSO be added).

5. [LocalMapping.cc](https://github.com/sunghoon031/LCSD_SLAM/blob/master/ORB_SLAM2/src/LocalMapping.cc): Performs joint geometric BA and checks the collinearity of the covisibility links between keyframes.
