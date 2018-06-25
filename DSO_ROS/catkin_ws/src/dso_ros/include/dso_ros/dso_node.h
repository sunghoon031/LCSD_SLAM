#ifndef DSO_ROS_DSO_NODE
#define DSO_ROS_DSO_NODE

#include <IOWrapper/Output3DWrapper.h>
#include <IOWrapper/Pangolin/PangolinDSOViewer.h>

#include <FullSystem/FullSystem.h>
#include <FullSystem/HessianBlocks.h>
#include <util/FrameShell.h>
#include <util/MinimalImage.h>
#include <util/Undistort.h>
#include <util/settings.h>
#include "util/DatasetReader.h"
#include "util/globalCalib.h"



#include <cv_bridge/cv_bridge.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Empty.h"
//#include <dso_ros/marginalized_msg.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <thread>
#include <iomanip>

#include <mutex>
#include <sys/stat.h>

//#include <tf/transform_broadcaster.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>


namespace dso_ros
{
class DsoNode
{
public:
  DsoNode(ros::NodeHandle& n, ros::NodeHandle& n_private);

  virtual ~DsoNode();

  //void imageCb(const sensor_msgs::ImageConstPtr& img);
  void KF_ID_calib_pose_points_callback(const std_msgs::Float32MultiArray& id_calib_pose_points);
  void Display_GUI_callback(const std_msgs::Empty &msg);
  void PublishMarginalizedStuffFromDSO(const float &publish_loop_rate);
  void RunDSO();

  sensor_msgs::ImageConstPtr ConvertImageAndExposure2ImagePtr(std::shared_ptr<dso::ImageAndExposure> input, bool half_resolution);

  void run()
  {
    ros::spin();
  }

//private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  //std::unique_ptr<dso::FullSystem> full_system;
  //std::unique_ptr<dso::Undistort> undistorter;

  ImageFolderReader* reader;
  dso::FullSystem* fullSystem;


  int mode; // 1=Run our system, 2=Generate rectified full-res img, 3=Generate rectified half-res img
  int dataset; //1=EuRoC MAV, 2=TUM monoVO
  int sequence; // sequence number for TUM monoVO

  int total_pub_count;

  int image_h, image_w, image_h_half, image_w_half;
  int marginalized_KF_ID;

  ros::Time start_time;

  double initial_timestamp;

  std::string image_file;
  std::string calib_file;
  std::string vignette_file;
  std::string gamma_file;
  std::string rectified_file;
  std::string stats_file;

  ros::Subscriber image_sub;
  ros::Subscriber KF_ID_calib_pose_points_sub;
  ros::Subscriber display_GUI_for_my_system_sub;

  ros::Publisher marginalized_KF_time_image_calib_pose_points_pub;
  ros::Publisher rectified_live_image_pub;
  ros::Publisher reset_count_pub;


  std_msgs::Int64 reset_count_msg;

  bool display_GUI; // This is for original DSO GUI
  bool display_GUI_for_my_system; // This is for my system
  bool half_resolution;
  bool preRectification;

  int original_setting_photometricCalibration;

  double playbackSpeed;
  int nSkippedFrames;
  int nTotalFramesAfterInit;
  std::vector<double> trackingTimes; //in ms
  double trackingStartTimestamp;
  double trackingEndTimestamp;
  double startTimestamp;
  double endTimestamp;

  void initParams();
  void reset();

  float setting_loop_rate;

  std::mutex frame_sync_mtx;

  std::vector<std::shared_ptr<dso::ImageAndExposure> > frame_ptrs_sync;
  std::vector<std::shared_ptr<dso::ImageAndExposure> > KF_ptrs_sync;


  std::vector<int> frame_IDs_sync, KF_IDs_sync, KF_IDs_temp;
  std::vector<double> frame_timestamps_sync, KF_timestamps_sync;
  std::vector<int> marginalized_KF_IDs_sync, marginalized_KF_IDs_temp;
  std::vector<float> marginalized_points_temp;
  std::vector<std::vector<float> > marginalized_KF_points_sync, marginalized_KF_points_temp;
  std::vector<std::vector<float> > marginalized_KF_poses_sync, marginalized_KF_poses_temp;
  std::vector<std::vector<float> > marginalized_KF_calib_sync, marginalized_KF_calib_temp;


};
}
#endif  // DSO_ROS_DSO_NODE

