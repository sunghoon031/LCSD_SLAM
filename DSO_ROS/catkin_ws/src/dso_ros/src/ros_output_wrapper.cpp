#include <dso_ros/ros_output_wrapper.h>

using namespace dso_ros;

//ROSOutputWrapper::ROSOutputWrapper(ros::NodeHandle& n,
//                                   ros::NodeHandle& n_private)
//{
////  ROS_INFO("ROSOutputWrapper created\n");
////  if (!n_private.hasParam("dso_frame_id")) {
////    ROS_WARN("No param named world_frame found!");
////  }
////  if (!n_private.hasParam("camera_frame_id")) {
////    ROS_WARN("No param named camera_frame found!");
////  }
////  n_private.param<std::string>("dso_frame_id", dso_frame_id_, "dso_odom");
////  n_private.param<std::string>("camera_frame_id", camera_frame_id_, "camera");
////  n_private.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
////  n_private.param<std::string>("base_frame_id", odom_frame_id_, "base_link");
////  ROS_INFO_STREAM("world_frame_id = " << dso_frame_id_ << "\n");
////  ROS_INFO_STREAM("camera_frame_id = " << camera_frame_id_ << "\n");
////  dso_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 5, false);
////  dso_depht_image_pub_ = n.advertise<sensor_msgs::Image>("image_rect", 5, false);
//
//  dso_KF_ID_pub= n.advertise<std_msgs::Int64MultiArray>("KF_ID", 10);
//  dso_marginalized_KF_ID_and_pose_pub = n.advertise<std_msgs::Float64MultiArray>("marginalized_KF_ID_and_pose", 10);
////  dso_marginalized_KF_image_pub = n.advertise<sensor_msgs::Image>("marginalized_KF_image", 10);
//  dso_marginalized_points_pub=n.advertise<std_msgs::Float64MultiArray>("marginalized_points",10);
//  dso_camera_intrinsics_pub=n.advertise<std_msgs::Float64MultiArray>("camera_intrinsics",10);
//}

ROSOutputWrapper::ROSOutputWrapper(ros::NodeHandle& n, ros::NodeHandle& n_private):
        wait_counter(0), oldest_active_KF_ID_prev(-1),oldest_KF_age(0), previously_skipped_KF_ID(-1)
{
    dso_KF_ID_calib_pose_points_pub = n.advertise<std_msgs::Float32MultiArray>("KF_ID_calib_pose_points", 10);
    dso_current_pose_pub = n.advertise<std_msgs::Float64MultiArray>("current_pose", 10);
    dso_active_poses_pub = n.advertise<std_msgs::Float64MultiArray>("active_poses", 10);
    dso_inlier_points_pub = n.advertise<std_msgs::Float64MultiArray>("inlier_points", 10);
    dso_full_stop_pub = n.advertise<std_msgs::Empty>("tracking_lost",1);

    GUI = false;

    dso_GUI_for_my_system_sub = n.subscribe("display_GUI_for_my_system", 10, &ROSOutputWrapper::GrabDisplayGUI, this);


}

ROSOutputWrapper::~ROSOutputWrapper()
{
  ROS_INFO("ROSOutputPublisher destroyed\n");
}

void ROSOutputWrapper::GrabDisplayGUI(const std_msgs::Empty &msg)
{
    GUI = true;
}

void ROSOutputWrapper::publishDsoKeyframeStuff(std::vector<dso::FrameHessian*>& frames,
                                        bool final, dso::CalibHessian* HCalib)
{
    std::vector<int> KF_ID_vec;
    std::vector<float> points;
    int marginalized_KF_ID = -1;
    std::vector<float> marginalized_pose(12,0.0);

    int oldest_active_KF_ID = -1;
    std::vector<float> oldest_active_pose(12,0.0);

    std_msgs::Float64MultiArray dso_active_poses_msg;
    std_msgs::Float64MultiArray dso_inlier_points_msg;

    int frame_count = 0;


    /*****************************************************/
    /*                                                   */
    /*    STEP 1. Collect estimation results from DSO.   */
    /*                                                   */
    /*****************************************************/

    for (dso::FrameHessian* f : frames) // For all active & marginalized frames
    {
        // [1] Collect each keyframe pose
        const Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> mTwc = f->shell->camToWorld.matrix3x4();

        for (size_t i =0, size = mTwc.size(); i< size; i++)
        {
            dso_active_poses_msg.data.push_back(*(mTwc.data()+i));
        }

        // [2] Collect the KF ID (if it is a new one)
        int KF_id = f->shell->incoming_id;

        auto iterator_ID = std::find(
                                KF_ID_vec.begin(),
                                KF_ID_vec.end(),
                               KF_id);

        if (iterator_ID == KF_ID_vec.end())
        {
            KF_ID_vec.push_back(KF_id);
        }

        if(KF_id > previously_skipped_KF_ID)
            frame_count++;

        // [3] Collect active & marginalized points
        std::vector<dso::PointHessian*> pointHessiansInliers = f->pointHessians;
        pointHessiansInliers.insert(
               pointHessiansInliers.end(),
               f->pointHessiansMarginalized.begin(),
               f->pointHessiansMarginalized.end());


        for (dso::PointHessian* p : pointHessiansInliers)
        {
           // Reject negative inverse depth (idepth_scaled = just idepth)
           if (p->idepth_scaled < 0)
               continue;

           float depth = 1.0f / p->idepth_scaled;
           float idepth_var = (1.0f / (p->idepth_hessian+0.01));
           float idepth_var_relative = idepth_var*depth*depth; // (= std_of_idepth / idepth)^2

           // Hardcoded setting from PangolinDSOViewer.cpp
           float my_scaledTH = 0.001;
           float my_absTH = 0.001;
           float my_minRelBS = 0.1;

           // Reject too high inverse depth variance (both relative and absolute)
           if(idepth_var_relative > my_scaledTH || idepth_var > my_absTH)
               continue;

           // Reject if the point was observed with too short baseline
           if(p->maxRelBaseline < my_minRelBS)
               continue;

           float u = p->u;
           float v = p->v;
           float color = p->color[0];
           float fx = HCalib->fxl();
           float fy = HCalib->fyl();
           float cx = HCalib->cxl();
           float cy = HCalib->cyl();

           float x_c = (u - cx)/fx*depth;
           float y_c = (v - cy)/fy*depth;
           float z_c = depth;

           float x_w =
                   mTwc(0,0)*x_c
                   + mTwc(0,1)*y_c
                   + mTwc(0,2)*z_c
                   + mTwc(0,3);
           float y_w =
                   mTwc(1,0)*x_c
                   + mTwc(1,1)*y_c
                   + mTwc(1,2)*z_c
                   +mTwc(1,3);
           float z_w =
                   mTwc(2,0)*x_c
                   + mTwc(2,1)*y_c
                   + mTwc(2,2)*z_c
                   + mTwc(2,3);

           points.push_back(x_w);
           points.push_back(y_w);
           points.push_back(z_w);
           points.push_back(idepth_var);


        }


        if (final) // if marginalized
        {
            // Collect the marginalized KF ID (if it is a new one) and pose as a vector of 13 elements
            // where the first element is the ID and the remaining 12 elements consist of the pose.

            marginalized_KF_ID = KF_id;

            //see: [https://stackoverflow.com/questions/16283000/most-efficient-way-to-loop-through-an-eigen-matrix]
            marginalized_pose.clear();
            for (size_t i =0, size = mTwc.size(); i< size; i++)
            {
                marginalized_pose.push_back(*(mTwc.data()+i));
            }

//            // Publish the image (WARNING: this is photometrically calibrated!)
//            cv::Mat image_cv = cv::Mat(dso::hG[0], dso::wG[0], CV_32FC3, f->dI->data())*(1/254.0f);
//            std::vector<cv::Mat> channels(3);
//            cv::split(image_cv, channels);
//            image_cv = channels[0];
//            image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
//            std_msgs::Header header;
//            header.stamp = ros::Time::now();
//            cv_bridge::CvImage bridge_img(header, "mono8", image_cv);
//            dso_marginalized_KF_image_pub_.publish(bridge_img.toImageMsg());
        }

        if (!final && frame_count==1)
        {
            oldest_active_KF_ID = KF_id;
            oldest_active_pose.clear();
            for (size_t i =0, size = mTwc.size(); i< size; i++)
            {
                oldest_active_pose.push_back(*(mTwc.data()+i));
            }


        }

    }


    /*****************************************************/
    /*                                                   */
    /*    STEP 2. Publish poses of all active KFs        */
    /*            and active points in the window        */
    /*            (FOR VISUALIZATION ONLY),              */
    /*                                                   */
    /*****************************************************/

    if (!final && GUI)
    {
        dso_active_poses_pub.publish(dso_active_poses_msg);

        dso_inlier_points_msg.data.insert(dso_inlier_points_msg.data.end(), points.begin(), points.end());
        dso_inlier_points_pub.publish(dso_inlier_points_msg);
    }

    /*****************************************************/
    /*                                                   */
    /*  STEP 3. Update the age of the oldest active KF   */
    /*          and check if we should wait for its      */
    /*          marginalization or just use the current  */
    /*          pose and points,                         */
    /*                                                   */
    /*****************************************************/

    if (!final && frames.size() == dso::setting_maxFrames+1 && !points.empty())
    {
        if (oldest_active_KF_ID == oldest_active_KF_ID_prev)
            oldest_KF_age++;
        else
        {
            oldest_KF_age = 0;
            oldest_active_KF_ID_prev = oldest_active_KF_ID;
        }

        if (oldest_KF_age == 5)
        {
            std::cout << "Oldest active keyframe's age = " << oldest_KF_age <<". Just use it even though not marginalized.." << std::endl;
            marginalized_KF_ID = oldest_active_KF_ID;
            marginalized_pose = oldest_active_pose;

            oldest_KF_age = 0;
            oldest_active_KF_ID_prev = oldest_active_KF_ID;
            previously_skipped_KF_ID = oldest_active_KF_ID;

        }
    }

    /*****************************************************/
    /*                                                   */
    /*    STEP 4. Publish all collected data to the      */
    /*            feature-based module                   */
    /*                                                   */
    /*****************************************************/

    // - 1st element = Number of KFs (=N)
    // - Next N elements = KF IDs
    // - Next element = marginalized KF ID (-1 if it doesn't exist)
    // - Next 4 elements = fx,fy,cx,cy
    // - Next 12 element = corresponding pose - either marginalized or most recent
    //   (going over each column of cam2world transformation matrix, Zeros if previous = -1)
    // - Remaining elements = (x_w, y_w, z_w) of each point

    std_msgs::Float32MultiArray dso_KF_ID_calib_pose_points_msg;

    std::sort(KF_ID_vec.begin(), KF_ID_vec.end());
    dso_KF_ID_calib_pose_points_msg.data.push_back(KF_ID_vec.size());
    dso_KF_ID_calib_pose_points_msg.data.insert(dso_KF_ID_calib_pose_points_msg.data.end(), KF_ID_vec.begin(), KF_ID_vec.end());
    dso_KF_ID_calib_pose_points_msg.data.push_back(marginalized_KF_ID);
    dso_KF_ID_calib_pose_points_msg.data.push_back(HCalib->fxl());
    dso_KF_ID_calib_pose_points_msg.data.push_back(HCalib->fyl());
    dso_KF_ID_calib_pose_points_msg.data.push_back(HCalib->cxl());
    dso_KF_ID_calib_pose_points_msg.data.push_back(HCalib->cyl());
    dso_KF_ID_calib_pose_points_msg.data.insert(dso_KF_ID_calib_pose_points_msg.data.end(), marginalized_pose.begin(), marginalized_pose.end());
    dso_KF_ID_calib_pose_points_msg.data.insert(dso_KF_ID_calib_pose_points_msg.data.end(), points.begin(), points.end());

    dso_KF_ID_calib_pose_points_pub.publish(dso_KF_ID_calib_pose_points_msg);

}

void ROSOutputWrapper::publishFullStop()
{
    dso_full_stop_pub.publish(std_msgs::Empty());
    std::cout << "Sending Full Stop signal !!" << std::endl;
}


void ROSOutputWrapper::publishCamPose(dso::FrameShell* frame,
                                      dso::CalibHessian* HCalib)
{
    // Seong addition
    if (GUI)
    {
        const Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> mTwc = frame->camToWorld.matrix3x4();

        std_msgs::Float64MultiArray dso_current_pose_msg;

        for (size_t i =0, size = mTwc.size(); i< size; i++)
        {
            dso_current_pose_msg.data.push_back(*(mTwc.data()+i));
        }

        dso_current_pose_pub.publish(dso_current_pose_msg);
    }




}

void ROSOutputWrapper::pushLiveFrame(dso::FrameHessian* image)
{
  // can be used to get the raw image / intensity pyramid.
}

void ROSOutputWrapper::pushDepthImage(dso::MinimalImageB3* image)
{
  // can be used to get the raw image with depth overlay.
}
bool ROSOutputWrapper::needPushDepthImage()
{
  return false;
}

void ROSOutputWrapper::pushDepthImageFloat(dso::MinimalImageF* image,
                                           dso::FrameHessian* KF)
{
//  cv::Mat image_cv(image->h, image->w, CV_32FC1, image->data);
//  image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
//  cv::Mat imverted_img;
//  cv::bitwise_not(image_cv, imverted_img);
//  std_msgs::Header header;
//  header.frame_id = camera_frame_id_;
//  header.stamp = ros::Time::now();
//  header.seq = seq_image_;
//  ++seq_image_;
//  cv_bridge::CvImage bridge_img(header, "mono8", imverted_img);
//  dso_depht_image_pub_.publish(bridge_img.toImageMsg());
}
