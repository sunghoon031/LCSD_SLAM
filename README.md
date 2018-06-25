#  **Loosely-Coupled Semi-Direct Monocular SLAM**

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


### 2.4. How to run: 

#### (1) Enable/Disable visualization (GUI):
&nbsp;&nbsp;&nbsp;&nbsp;(a) GUI for the direct module (DSO): Go to `dso_ros/catkin_ws/src/dso_ros/launch/euroc_***.launch` (for the EuRoC MAV dataset) or `dso_ros/catkin_ws/src/dso_ros/launch/monoVO_***.launch` (for the TUM monoVO dataset) and set `display_GUI` to either true or false.

&nbsp;&nbsp;&nbsp;&nbsp;(b) GUI for the feature-based module (ORB-SLAM): Go to `ORB_SLAM2/Examples/Monocular/EuRoC_seong_***.yaml` (for the EuRoC MAV dataset) or `ORB_SLAM2/Examples/Monocular/TUMmonoVO_yaml/TUM_monoVO_***.yaml` (for the TUM monoVO dataset) and set `GUI` to either 1 or 0.

Note that enabling both GUIs can slow down the performance. The results reported in the paper were obtained by running the system with both GUIs disabled. If you really want the optimal result with the visualization, reduce the playback speed by half (see next) and enable the GUI of ORB-SLAM only. 

#### (2) Set the playback speed:
This can be done by setting the parameter value of `playback_speed` in the launch file (see [Step 1](https://github.com/sunghoon031/LSV-SLAM/blob/master/README.md#3-set-paths)). The default is 1 (original speed).

#### (3) Set paths:
Open `dso_ros/catkin_ws/src/dso_ros/launch/euroc_seong_***.launch` (for the EuRoC MAV dataset) or `dso_ros/catkin_ws/src/dso_ros/launch/monoVO_***.launch` (for the TUM monoVO dataset) and edit the input paths (`image_file_path`, `calib_file_path`, `vignette_file_path`, `gamma_file_path`) and the output path (`stats_file_path`).

#### (4) Run:
&nbsp;&nbsp;&nbsp;&nbsp;(i) In the first terminal, run
````
// For the EuRoC MAV dataset:
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt \
Examples/Monocular/EuRoC_seong_[VO/SLAM]_cam[0/1].yaml 

// For the TUM monoVO dataset:
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt \
Examples/Monocular/TUMmonoVO_yaml/monoVO_ORB_[VO/SLAM]_full_[SEQUENCE_NUMBER].yaml 
````
&nbsp;&nbsp;&nbsp;&nbsp;(ii) In the second terminal, run
````
cd [APPROPRIATE_PATH]/dso_ros/catkin_ws
source devel/setup.bash

// For the EuRoC MAV dataset:
roslaunch dso_ros EuRoC_seong_[SEQUENCE_NAME]_cam[0/1].launch

// For the TUM monoVO dataset:
roslaunch dso_ros monoVO_seong_[SEQUENCE_NUMBER].launch
````
The keyframe trajectory will be saved as `KeyFrameTrajectory_seong.txt` in `ORB_SLAM2` folder, and additional tracking statistics will be saved as `trackingStats.txt` in the path you set (`stats_file_path`) in [Step 3](https://github.com/sunghoon031/LSV-SLAM/blob/master/README.md#3-set-paths).
