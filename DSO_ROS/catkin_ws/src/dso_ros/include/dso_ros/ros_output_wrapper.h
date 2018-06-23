/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DSO_ROS_ROS_IO_WRAPPER
#define DSO_ROS_ROS_IO_WRAPPER

#include <FullSystem/HessianBlocks.h>
#include <IOWrapper/Output3DWrapper.h>
#include <util/FrameShell.h>
#include <util/MinimalImage.h>

#include <Eigen/Core>

#include <boost/thread.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Empty.h"

//#include "geometry_msgs/TransformStamped.h"
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>

#include <dso_ros/dso_node.h>


namespace dso_ros{

class ROSOutputWrapper : public dso::IOWrap::Output3DWrapper
{
public:
  ROSOutputWrapper(ros::NodeHandle& n, ros::NodeHandle& n_private);

  virtual ~ROSOutputWrapper();

  virtual void publishDsoKeyframeStuff(
          std::vector<dso::FrameHessian*>& frames,
          bool final,
          dso::CalibHessian* HCalib
          ) override;

  virtual void publishFullStop() override;

  virtual void publishCamPose(dso::FrameShell* frame,
                              dso::CalibHessian* HCalib) override;

  virtual void pushLiveFrame(dso::FrameHessian* image) override;

  virtual void pushDepthImage(dso::MinimalImageB3* image) override;

  virtual bool needPushDepthImage() override;

  virtual void pushDepthImageFloat(dso::MinimalImageF* image,
                                   dso::FrameHessian* KF) override;

  void GrabDisplayGUI(const std_msgs::Empty &msg);

//private:
  /* camera frame id */
//  std::string dso_frame_id_;
//  std::string camera_frame_id_;
//  std::string odom_frame_id_;
//  std::string base_frame_id_;
//  /* dso odometry_msg */
//  ros::Publisher dso_odom_pub_;
//  ros::Publisher dso_depht_image_pub_;
//  tf::TransformListener tf_list_;
//  tf::TransformBroadcaster tf_broadcast_;
  //  ros::Publisher dso_marginalized_KF_image_pub;


  ros::Publisher dso_KF_ID_calib_pose_points_pub;
  ros::Publisher dso_active_poses_pub;
  ros::Publisher dso_inlier_points_pub;
  ros::Publisher dso_current_pose_pub;
  ros::Publisher dso_full_stop_pub;

  ros::Subscriber dso_GUI_for_my_system_sub;

  bool GUI;
  int wait_counter;
  int oldest_active_KF_ID_prev;
  int oldest_KF_age;
  int previously_skipped_KF_ID;

};
}
#endif  // DSO_ROS_ROS_IO_WRAPPER

