/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<math.h>
#include<vector>
#include<deque>
#include<thread>
#include<random>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Empty.h"

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;

    //ros::Publisher pub_land;
    //bool pub_land_sent = false;

};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //TEMP
//    if (!pub_land_sent && !cv_ptr->image.empty())
//    {
//        std::cout << "land published!" << std::endl;
//        pub_land.publish(std_msgs::Empty());
//        pub_land_sent = true;
//    }
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


class rosWrapper
{
public:
    rosWrapper(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabDsoResetCount(const std_msgs::Int64 &count);
    void GrabDsoMarinalizedKeyframeInfo(const std_msgs::Float64MultiArray &marginalized_info);
    void GrabDsoCurrentPose(const std_msgs::Float64MultiArray &current_pose);
    void GrabDsoActiveKFsPoses(const std_msgs::Float64MultiArray &active_poses);
    void GrabDsoInlierPoints(const std_msgs::Float64MultiArray &inlier_points);
    void GrabDsoFullStop(const std_msgs::Empty &msg);
    void RunDsoTracker(const float &tracker_loop_rate);
    void ComputeUVZVARandDepthsMapFromDSO(
            const std::vector<float> &calib_from_dso,
            const cv::Mat &Rcw_from_dso,
            const cv::Mat &tcw_from_dso,
            const std::vector<cv::Mat> &points_from_dso);
    void ComputeSim3Alignment(
            const cv::Mat &dso_positions_,
            const cv::Mat &orb_positions_,
            const cv::Mat &latest_Tcw_dso_,
            const cv::Mat &latest_Tcw_orb_,
            const bool &symmetry_,
            const bool &hold_rotation_and_scale_);
    void GetStartEndTimstampsWithGroundTruth(const int &seq);
    bool DisableKeyframeCulling(const double &timestamp);
    void Reset();

    ORB_SLAM2::System* mpSLAM;

    ros::Publisher display_GUI_for_my_system_pub;

    std::mutex sync_mtx;
    std::mutex dso_cam_mtx;

    std::vector<double> marginalized_KF_timestamps_temp, marginalized_KF_timestamps_sync;
    std::vector<cv::Mat> marginalized_KF_images_temp, marginalized_KF_images_sync;
    std::vector<std::vector<float> > marginalized_KF_calib_temp, marginalized_KF_calib_sync;
    std::vector<cv::Mat> marginalized_KF_Twc_matrices_temp, marginalized_KF_Twc_matrices_sync;
    std::vector<std::vector<cv::Mat> > marginalized_KF_points_temp, marginalized_KF_points_sync;


    bool display_GUI;
    bool DSO_reset_called = false;
    bool DSO_full_stop_called = false;
    bool lost_at_least_once = false;
    unsigned nKFs_in_map_when_lost = 0;

    bool Sim3Aligned = false;

    bool temp_clear_signal_received = false;
    bool sync_clear_signal_received = false;

    cv::Mat Twc_dso_cam = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Tcw_prev_dso = cv::Mat::eye(4,4,CV_32F);
    cv::Mat orb_Tcw_final = cv::Mat::eye(4,4,CV_32F);
    cv::Mat dso_positions, orb_positions;
    cv::Mat dso_recent_positions, orb_recent_positions;

    // Sim(3) Alignment between orb-world and dso-world
    cv::Mat R_w_orb_from_w_dso = cv::Mat::eye(3,3,CV_32F);
    cv::Mat t_w_orb_from_w_dso = cv::Mat::ones(3,1,CV_32F);
    float scale_w_orb_from_w_dso = 1;
    cv::Mat S_w_orb_from_w_dso = cv::Mat::eye(4,4,CV_32F);
    cv::Mat S_w_dso_from_w_orb = cv::Mat::eye(4,4,CV_32F);

    int dso_reset_count_prev;
    int total_sub_count;
    int dso_kf_count;
    int orb_kf_count;
    int scale_jump_count;

    unsigned nPositions_for_scale = 10;

    bool nfeatures_fixed;
    bool TUM_monoVO;
    bool disableKFculling_prev;

    std::vector<double> tsNoGroundTruth; // size 100 = (end timestamp for start-segment & start timestamp for end-segment) x 50 sequences
    std::vector<double> time_btwn_track_vec;
    double timestamp_prev, timestamp_init;
    double KF_rate;

    float setting_loop_rate = 10; //Hz
    int setting_KF_addition_minMatches = 150;
    int setting_KF_addition_maxFrames = 4;// Make at least one KF out of this number.
    float setting_response_thr_factor = 0;
    int setting_max_DSO_points_addition = 10000;
    bool setting_add_DSO_points_nonKF = false;
    bool setting_add_DSO_points_KF = false;


    int img_width, img_height;
    std::vector<float> depths_map_2send;
    std::vector<cv::Mat> uvzvar_2send;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Read the setting
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    int Mode = fSettings["Mode"];
    int GUI = fSettings["GUI"];
    int Dataset = fSettings["Dataset"];
    bool DisplayGUI = (GUI == 1 ? true : false);



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, DisplayGUI);




    if (Mode == 1) // No Loop Closure
    {
        std::cout << "!! NO LOOP CLOSURE !!" << std::endl;
        SLAM.mpLoopCloser->DisableLoopClosure = true;
    }
    else
    {
        std::cout << "!! WITH LOOP CLOSURE !!" << std::endl;
        SLAM.mpLoopCloser->DisableLoopClosure = false;
    }

    rosWrapper rosWrap(&SLAM);

    rosWrap.display_GUI = DisplayGUI;
    rosWrap.TUM_monoVO =  (Dataset == 2 ? true : false);
    rosWrap.KF_rate = 0;
    rosWrap.nfeatures_fixed = false;
    rosWrap.orb_kf_count = 0;
    rosWrap.dso_kf_count = 0;
    rosWrap.scale_jump_count = 0;
    rosWrap.scale_w_orb_from_w_dso = 1;

    if (rosWrap.TUM_monoVO)
    {
        int sequence = fSettings["Sequence"];
        rosWrap.GetStartEndTimstampsWithGroundTruth(sequence);
    }


    // With or without relocalization or loop closure? Set it here!
    SLAM.mpTracker->DisableRelocalization = true;
    SLAM.mpTracker->GUI = DisplayGUI;
    SLAM.mpTracker->KF_addition_minMatches = rosWrap.setting_KF_addition_minMatches;
    SLAM.mpTracker->KF_addition_maxFrames = rosWrap.setting_KF_addition_maxFrames;
    SLAM.mpTracker->add_DSO_points_nonKF = rosWrap.setting_add_DSO_points_nonKF;
    SLAM.mpTracker->add_DSO_points_KF = rosWrap.setting_add_DSO_points_KF;
    SLAM.mpTracker->max_DSO_points_addition = rosWrap.setting_max_DSO_points_addition;

    SLAM.mpTracker->TUM_monoVO = rosWrap.TUM_monoVO;
    SLAM.mpLocalMapper->TUM_monoVO = rosWrap.TUM_monoVO;
    SLAM.mpLocalMapper->DisableKeyframeCulling = false;


    ros::NodeHandle nodeHandler;

    ros::Subscriber dso_reset_count_sub = nodeHandler.subscribe("/dso/reset", 10, &rosWrapper::GrabDsoResetCount,&rosWrap);
    ros::Subscriber dso_marginalized_KF_info_sub = nodeHandler.subscribe("/dso/marginalized_KF_time_image_calib_pose_points", 10, &rosWrapper::GrabDsoMarinalizedKeyframeInfo,&rosWrap);
    ros::Subscriber dso_tracking_lost_sub = nodeHandler.subscribe("/dso/tracking_lost", 10, &rosWrapper::GrabDsoFullStop,&rosWrap);
    ros::Subscriber dso_current_pose_sub = nodeHandler.subscribe("/dso/current_pose", 10, &rosWrapper::GrabDsoCurrentPose,&rosWrap);
    ros::Subscriber dso_active_poses_sub = nodeHandler.subscribe("/dso/active_poses", 10, &rosWrapper::GrabDsoActiveKFsPoses,&rosWrap);
    ros::Subscriber dso_inlier_points_sub = nodeHandler.subscribe("/dso/inlier_points", 10, &rosWrapper::GrabDsoInlierPoints,&rosWrap);

    rosWrap.display_GUI_for_my_system_pub = nodeHandler.advertise<std_msgs::Empty>("/dso/display_GUI_for_my_system",1);


    std::thread trackerThread (&rosWrapper::RunDsoTracker, &rosWrap, rosWrap.setting_loop_rate);

    ros::spin();

//    // Stop all threads
//    SLAM.Shutdown();
//
//    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_seong.txt");
//
//    ros::shutdown();

    trackerThread.join();






    return 0;
}


void rosWrapper::GrabDsoCurrentPose(const std_msgs::Float64MultiArray &current_pose)
{
    std::vector<double> pose_vec = current_pose.data;

    // For visualization of the current camera pose:

    if (mpSLAM->mpMap->KeyFramesInMap() > 2)
    {
        cv::Mat Twc_dso_cam_copied;
        {
            std::unique_lock<std::mutex> lock (dso_cam_mtx);
            Twc_dso_cam =(cv::Mat_<float>(4,4) <<
                            pose_vec[0], pose_vec[3], pose_vec[6], pose_vec[9],
                            pose_vec[1], pose_vec[4], pose_vec[7], pose_vec[10],
                            pose_vec[2], pose_vec[5], pose_vec[8], pose_vec[11],
                            0, 0, 0, 1);
            Twc_dso_cam_copied = Twc_dso_cam.clone();
        }

        cv::Mat Rwc_dso_cam = Twc_dso_cam_copied.rowRange(0,3).colRange(0,3);
        cv::Mat Rcw_dso_cam = Rwc_dso_cam.t();
        cv::Mat twc_dso_cam = Twc_dso_cam_copied.rowRange(0,3).col(3);
        cv::Mat tcw_dso_cam = -Rcw_dso_cam*twc_dso_cam;
        cv::Mat Tcw_dso_cam = cv::Mat::eye(4,4,CV_32F);
        Rcw_dso_cam.copyTo(Tcw_dso_cam.rowRange(0,3).colRange(0,3));
        tcw_dso_cam.copyTo(Tcw_dso_cam.rowRange(0,3).col(3));

        cv::Mat Tcw_dso_current_in_orb = cv::Mat::eye(4,4,CV_32F);
        cv::Mat Twc_dso_current_in_orb = cv::Mat::eye(4,4,CV_32F);

        if (mpSLAM->mpMap->IsOrbPoseFinal())
        {
            if (!Sim3Aligned)
            {
                cv::Mat Tcw_last_kf = mpSLAM->mpMap->GetLastGoodKeyFrame()->GetPose();
                cv::Mat Twc_dso_last_kf = mpSLAM->mpMap->GetLastGoodKeyFrame()->GetDsoPoseInverse();
                cv::Mat Twc_last_kf = mpSLAM->mpMap->GetLastGoodKeyFrame()->GetPoseInverse();
                cv::Mat Tcw_dso_last_kf = mpSLAM->mpMap->GetLastGoodKeyFrame()->GetDsoPose();

                Tcw_dso_current_in_orb = Tcw_dso_cam*Twc_dso_last_kf*Tcw_last_kf;
                Twc_dso_current_in_orb = Twc_last_kf*Tcw_dso_last_kf*Twc_dso_cam_copied;
            }
            else
            {
                Tcw_dso_current_in_orb = Tcw_dso_cam*S_w_dso_from_w_orb;
                Twc_dso_current_in_orb = S_w_orb_from_w_dso*Twc_dso_cam_copied;
            }
        }
        else
        {
            Twc_dso_current_in_orb = Twc_dso_cam_copied;
            Tcw_dso_current_in_orb = Tcw_dso_cam;
        }

        mpSLAM->mpMapDrawer->SetCurrentDsoCameraPose(Twc_dso_current_in_orb,Tcw_dso_current_in_orb);
    }
}

void rosWrapper::GrabDsoActiveKFsPoses(const std_msgs::Float64MultiArray &active_poses)
{
    std::vector<double> pose_vec = active_poses.data;
    std::vector<cv::Mat> Twc_dso_kf_vec;

    if (mpSLAM->mpMap->KeyFramesInMap() > 2 )
    {
        bool updated = false;
        while(!pose_vec.empty())
        {
            updated = true;
            cv::Mat Twc_dso_kf =(cv::Mat_<float>(4,4) <<
                        pose_vec[0], pose_vec[3], pose_vec[6], pose_vec[9],
                        pose_vec[1], pose_vec[4], pose_vec[7], pose_vec[10],
                        pose_vec[2], pose_vec[5], pose_vec[8], pose_vec[11],
                        0, 0, 0, 1);

            cv::Mat Twc_dso_kf_in_orb;

            if (mpSLAM->mpMap->IsOrbPoseFinal())
            {
                if (!Sim3Aligned)
                {
                    cv::Mat Tcw_dso_current_in_orb = mpSLAM->mpMapDrawer->GetCurrentDsoCameraPose().at(1);
                    if (Tcw_dso_current_in_orb.empty())
                        break;

                    cv::Mat Twc_dso_cam_copied;
                    {
                        std::unique_lock<std::mutex> lock (dso_cam_mtx);
                        Twc_dso_cam_copied = Twc_dso_cam.clone();
                    }

                    cv::Mat Rwc_dso_cam = Twc_dso_cam_copied.rowRange(0,3).colRange(0,3);
                    cv::Mat Rcw_dso_cam = Rwc_dso_cam.t();
                    cv::Mat twc_dso_cam = Twc_dso_cam_copied.rowRange(0,3).col(3);
                    cv::Mat tcw_dso_cam = -Rcw_dso_cam*twc_dso_cam;
                    cv::Mat Tcw_dso_cam = cv::Mat::eye(4,4,CV_32F);
                    Rcw_dso_cam.copyTo(Tcw_dso_cam.rowRange(0,3).colRange(0,3));
                    tcw_dso_cam.copyTo(Tcw_dso_cam.rowRange(0,3).col(3));

                    cv::Mat Rcw_dso_current_in_orb = Tcw_dso_current_in_orb.rowRange(0,3).colRange(0,3);
                    cv::Mat Rwc_dso_current_in_orb = Rcw_dso_current_in_orb.t();
                    cv::Mat tcw_dso_current_in_orb = Tcw_dso_current_in_orb.rowRange(0,3).col(3);
                    cv::Mat twc_dso_current_in_orb = -Rwc_dso_current_in_orb*tcw_dso_current_in_orb;
                    cv::Mat Twc_dso_current_in_orb = cv::Mat::eye(4,4,CV_32F);
                    Rwc_dso_current_in_orb.copyTo(Twc_dso_current_in_orb.rowRange(0,3).colRange(0,3));
                    twc_dso_current_in_orb.copyTo(Twc_dso_current_in_orb.rowRange(0,3).col(3));

                    Twc_dso_kf_in_orb = Twc_dso_current_in_orb*Tcw_dso_cam*Twc_dso_kf;
                }
                else
                {
                    Twc_dso_kf_in_orb = S_w_orb_from_w_dso*Twc_dso_kf;
                }

                Twc_dso_kf_vec.push_back(Twc_dso_kf_in_orb);
            }
            else
                Twc_dso_kf_vec.push_back(Twc_dso_kf);

            pose_vec.erase(pose_vec.begin(), pose_vec.begin() + 12);

        }

        if (updated)
            mpSLAM->mpMap->SetDsoKFsPoses(Twc_dso_kf_vec);
    }
}

void rosWrapper::GrabDsoInlierPoints(const std_msgs::Float64MultiArray &inlier_points)
{
    std::vector<double> points_vec = inlier_points.data;

    std::vector<cv::Mat> points;
    while (points_vec.size() > 0)
    {
        std::vector<float> point_vec(points_vec.begin(),points_vec.begin()+4);
        points_vec.erase(points_vec.begin(),points_vec.begin()+4);
        points.push_back((cv::Mat_<float>(4,1)<< point_vec[0], point_vec[1], point_vec[2], point_vec[3]));
    }

    if (mpSLAM->mpMap->KeyFramesInMap() > 2 && !points.empty())
    {
        mpSLAM->mpMap->ClearDsoMapPointsBeforeSim3Alignment();
        mpSLAM->mpMap->ClearDsoMapPointsAfterSim3Alignment();

        for (auto point : points)
        {
            cv::Mat point_DSO_w =(cv::Mat_<float>(3,1) << point.at<float>(0),point.at<float>(1),point.at<float>(2));

            if (mpSLAM->mpMap->IsOrbPoseFinal())
            {
                if (!Sim3Aligned)
                {
                    cv::Mat Rcw_dso_last_kf = mpSLAM->mpMap->GetLastGoodKeyFrame()->GetDsoRotation();
                    cv::Mat tcw_dso_last_kf = mpSLAM->mpMap->GetLastGoodKeyFrame()->GetDsoTranslation();
                    cv::Mat point_DSO_c = Rcw_dso_last_kf*point_DSO_w + tcw_dso_last_kf;

                    // Cheap trick: Instead of initializing with world position, use position in camera frame!
                    ORB_SLAM2::MapPoint* pNewMP = new ORB_SLAM2::MapPoint(point_DSO_c,mpSLAM->mpMap->GetLastGoodKeyFrame(),mpSLAM->mpMap);
                    mpSLAM->mpMap->AddDsoMapPointBeforeSim3Alignment(pNewMP);
                }
                else
                {
                    cv::Mat point_ORB_w = scale_w_orb_from_w_dso*R_w_orb_from_w_dso*point_DSO_w + t_w_orb_from_w_dso;
                    ORB_SLAM2::MapPoint* pNewMP = new ORB_SLAM2::MapPoint(point_ORB_w,mpSLAM->mpMap->GetLastGoodKeyFrame(),mpSLAM->mpMap);
                    mpSLAM->mpMap->AddDsoMapPointAfterSim3Alignment(pNewMP);
                }
            }
            else
            {
                ORB_SLAM2::MapPoint* pNewMP = new ORB_SLAM2::MapPoint(point_DSO_w,mpSLAM->mpMap->GetLastGoodKeyFrame(),mpSLAM->mpMap);
                mpSLAM->mpMap->AddDsoMapPointAfterSim3Alignment(pNewMP);
            }


        }

    }

}





void rosWrapper::GrabDsoMarinalizedKeyframeInfo(const std_msgs::Float64MultiArray &marginalized_info)
{
    if (temp_clear_signal_received)
    {
        temp_clear_signal_received = false;
        marginalized_KF_timestamps_temp.clear();
        marginalized_KF_images_temp.clear();
        marginalized_KF_calib_temp.clear();
        marginalized_KF_Twc_matrices_temp.clear();
        marginalized_KF_points_temp.clear();
    }

    if (sync_clear_signal_received)
    {
        sync_clear_signal_received = false;
        std::unique_lock<std::mutex> lock (sync_mtx);
        marginalized_KF_timestamps_sync.clear();
        marginalized_KF_images_sync.clear();
        marginalized_KF_calib_sync.clear();
        marginalized_KF_Twc_matrices_sync.clear();
        marginalized_KF_points_sync.clear();
    }

    // 1st element: timestamp
    // 2nd & 3rd element: h & w
    // Next elements: image
    // Next elemetns: fx,fy,cx,cy
    // Next elements: pose
    // Next elements: points:

    std::vector<double> marginalized_info_received = marginalized_info.data;

    // Get timestamp
    marginalized_KF_timestamps_temp.push_back(marginalized_info_received.front());
    marginalized_info_received.erase(marginalized_info_received.begin());

    // Get image
    int h = (int) marginalized_info_received.front();
    marginalized_info_received.erase(marginalized_info_received.begin());
    int w = (int) marginalized_info_received.front();
    marginalized_info_received.erase(marginalized_info_received.begin());

    img_height = h;
    img_width = w;

    std::vector<float> image_vector(
            marginalized_info_received.begin(),
            marginalized_info_received.begin()+h*w);
    assert(marginalized_info_received.size() > h*w);

    marginalized_info_received.erase(
            marginalized_info_received.begin(),
            marginalized_info_received.begin()+h*w);

    cv::Mat image_cv = cv::Mat(h, w, CV_32FC1, image_vector.data())*(1/254.0f);
    image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
    marginalized_KF_images_temp.push_back(image_cv);

    // Get calibration
    std::vector<float> calib_vec(marginalized_info_received.begin(),marginalized_info_received.begin()+4);
    marginalized_info_received.erase(marginalized_info_received.begin(),marginalized_info_received.begin()+4);
    marginalized_KF_calib_temp.push_back(calib_vec);

    // Get pose
    std::vector<float> pose_vec(marginalized_info_received.begin(),marginalized_info_received.begin()+12);
    marginalized_info_received.erase(marginalized_info_received.begin(),marginalized_info_received.begin()+12);

    cv::Mat Twc_dso =(cv::Mat_<float>(4,4) <<
            pose_vec[0], pose_vec[3], pose_vec[6], pose_vec[9],
            pose_vec[1], pose_vec[4], pose_vec[7], pose_vec[10],
            pose_vec[2], pose_vec[5], pose_vec[8], pose_vec[11],
            0, 0, 0, 1);
    Twc_dso.convertTo(Twc_dso, CV_32F);
    marginalized_KF_Twc_matrices_temp.push_back(Twc_dso);


    // Get Points (x_w, y_w, z_w, idepth_var_relative)
    assert(marginalized_info_received.size() % 4 == 0);

    std::vector<cv::Mat> points;
    while (marginalized_info_received.size() > 0)
    {
        std::vector<float> point_vec(
                marginalized_info_received.begin(),
                marginalized_info_received.begin()+4);
        marginalized_info_received.erase(
                marginalized_info_received.begin(),
                marginalized_info_received.begin()+4);
        points.push_back(
                    (cv::Mat_<float>(4,1)
                    << point_vec[0], point_vec[1], point_vec[2], point_vec[3]));
    }


    assert(marginalized_info_received.size() == 0);

    marginalized_KF_points_temp.push_back(points);

    if (!marginalized_KF_timestamps_temp.empty()
                 && !marginalized_KF_images_temp.empty()
                 && !marginalized_KF_calib_temp.empty()
                 && !marginalized_KF_Twc_matrices_temp.empty()
                 && !marginalized_KF_points_temp.empty())
     {
        std::unique_lock<std::mutex> lock (sync_mtx);
        marginalized_KF_timestamps_sync.push_back(marginalized_KF_timestamps_temp.front());
        marginalized_KF_images_sync.push_back(marginalized_KF_images_temp.front());
        marginalized_KF_calib_sync.push_back(marginalized_KF_calib_temp.front());
        marginalized_KF_Twc_matrices_sync.push_back(marginalized_KF_Twc_matrices_temp.front());
        marginalized_KF_points_sync.push_back(marginalized_KF_points_temp.front());

        marginalized_KF_timestamps_temp.erase(marginalized_KF_timestamps_temp.begin());
        marginalized_KF_images_temp.erase(marginalized_KF_images_temp.begin());
        marginalized_KF_calib_temp.erase(marginalized_KF_calib_temp.begin());
        marginalized_KF_Twc_matrices_temp.erase(marginalized_KF_Twc_matrices_temp.begin());
        marginalized_KF_points_temp.erase(marginalized_KF_points_temp.begin());
     }
}



void rosWrapper::GrabDsoResetCount(const std_msgs::Int64 &count)
{
    if (dso_reset_count_prev != count.data) // If DSO was reset
    {
        std::cout << "!! DSO was reset (counter = "<< count.data <<")!!" <<std::endl;
        dso_reset_count_prev = count.data;
        DSO_reset_called = true;
    }

}

void rosWrapper::GrabDsoFullStop(const std_msgs::Empty &msg)
{
    DSO_full_stop_called = true;
}

void rosWrapper::RunDsoTracker(const float &tracker_loop_rate)
{
    ros::Rate loop_rate(tracker_loop_rate);
    int wait_counter = 0;

    while (ros::ok())
    {
        loop_rate.sleep();

        if (display_GUI)
            display_GUI_for_my_system_pub.publish(std_msgs::Empty());

        if (DSO_reset_called /*|| !DSO_marginalized_after_ORB_init*/)
        {
            Reset();
            continue;
        }

        if (!marginalized_KF_timestamps_sync.empty()
                && !marginalized_KF_images_sync.empty()
                && !marginalized_KF_calib_sync.empty()
                && !marginalized_KF_Twc_matrices_sync.empty()
                && !marginalized_KF_points_sync.empty())
        {

            double KF_timestamp;
            cv::Mat KF_image;
            std::vector<float> calib;
            cv::Mat Twc_dso;
            std::vector<cv::Mat> KF_points;

            {
                std::unique_lock<std::mutex> lock (sync_mtx);

                KF_timestamp = marginalized_KF_timestamps_sync.front();
                KF_image = marginalized_KF_images_sync.front().clone();
                calib = marginalized_KF_calib_sync.front();
                Twc_dso = marginalized_KF_Twc_matrices_sync.front().clone();
                KF_points = marginalized_KF_points_sync.front();
                marginalized_KF_timestamps_sync.erase(marginalized_KF_timestamps_sync.begin());
                marginalized_KF_images_sync.erase(marginalized_KF_images_sync.begin());
                marginalized_KF_calib_sync.erase(marginalized_KF_calib_sync.begin());
                marginalized_KF_Twc_matrices_sync.erase(marginalized_KF_Twc_matrices_sync.begin());
                marginalized_KF_points_sync.erase(marginalized_KF_points_sync.begin());
            }

            wait_counter = 0;

            cv::Mat Rwc_dso = Twc_dso.rowRange(0,3).colRange(0,3);
            cv::Mat Rcw_dso = Rwc_dso.t();
            cv::Mat twc_dso = Twc_dso.rowRange(0,3).col(3);
            cv::Mat tcw_dso = -Rcw_dso*twc_dso;

            cv::Mat Tcw_dso = cv::Mat::eye(4,4,CV_32F);
            Rcw_dso.copyTo(Tcw_dso.rowRange(0,3).colRange(0,3));
            tcw_dso.copyTo(Tcw_dso.rowRange(0,3).col(3));


            // Compute incremental transformation:
            cv::Mat R_inc_dso = Rcw_dso*(Tcw_prev_dso.rowRange(0,3).colRange(0,3)).t();
            cv::Mat t_inc_dso = -R_inc_dso*(Tcw_prev_dso.rowRange(0,3).col(3)) + tcw_dso;
            t_inc_dso *= scale_w_orb_from_w_dso;

            cv::Mat T_inc_dso = cv::Mat::eye(4,4,CV_32F);
            R_inc_dso.copyTo(T_inc_dso.rowRange(0,3).colRange(0,3));
            t_inc_dso.copyTo(T_inc_dso.rowRange(0,3).col(3));

            Tcw_prev_dso = Tcw_dso.clone();

            // Send the corresponding points
            ComputeUVZVARandDepthsMapFromDSO(calib, Rcw_dso, tcw_dso, KF_points);

            if (TUM_monoVO)
            {
                //mpSLAM->mpTracker->add_DSO_points_KF = true;
                disableKFculling_prev = mpSLAM->mpLocalMapper->DisableKeyframeCulling;
                mpSLAM->mpLocalMapper->DisableKeyframeCulling = DisableKeyframeCulling(KF_timestamp);

                if (disableKFculling_prev != mpSLAM->mpLocalMapper->DisableKeyframeCulling)
                {
                    if (mpSLAM->mpLocalMapper->DisableKeyframeCulling)
                        std::cout << std::endl << "!! Keyframe Culling Disabled !!" << std::endl << std::endl;
                    else
                        std::cout << std::endl << "!! Keyframe Culling Enabled !!" << std::endl << std::endl;

                }

            }




            if (KF_rate > 0.1)
            {
                double freq_low = 4;
                double freq_high = 7;
                double freq_max = 8;

                int nfeatures = 2500;

                if (!nfeatures_fixed && KF_rate > freq_max)
                {
                    nfeatures_fixed = true;
                    mpSLAM->mpTracker->add_DSO_points_KF = true;
                    delete mpSLAM->mpTracker->mpORBextractorLeft;
                    mpSLAM->mpTracker->mpORBextractorLeft = new ORB_SLAM2::ORBextractor(1500,1.2,8,20,7);
                }

                if (!nfeatures_fixed)
                {
                    if (KF_rate < freq_low)
                    {
                        nfeatures = 2500;
                        mpSLAM->mpTracker->add_DSO_points_KF = false;
                    }
                    else if (KF_rate > freq_high)
                    {
                        nfeatures = 1500;
                        mpSLAM->mpTracker->add_DSO_points_KF = true;
                    }
                    else
                    {
                        nfeatures = (int)(2500-1000*(KF_rate-freq_low)/(freq_high-freq_low));
                        mpSLAM->mpTracker->add_DSO_points_KF = false;
                    }

                    delete mpSLAM->mpTracker->mpORBextractorLeft;
                    mpSLAM->mpTracker->mpORBextractorLeft = new ORB_SLAM2::ORBextractor(nfeatures,1.2,8,20,7);

                    std::cout << "nfeatures = " << nfeatures << std::endl;
                }


            }
            else if (KF_rate == 0 && mpSLAM->mpTracker->mpORBextractorLeft->nfeatures != 2500)
            {
                delete mpSLAM->mpTracker->mpORBextractorLeft;
                mpSLAM->mpTracker->mpORBextractorLeft = new ORB_SLAM2::ORBextractor(2500,1.2,8,20,7);
                std::cout << "nfeatures init = 2500" << std::endl;
            }


            // Wait until local mapping is idle, or loop closure is complete.
            while (mpSLAM->mpLocalMapper->CheckNewKeyFrames()
                    || mpSLAM->mpLocalMapper->stopRequested()
                    || mpSLAM->mpLocalMapper->isStopped())
                usleep(0.01*1000*1000);

            // Skipping the first 10 KFs for computing KF_rate
            dso_kf_count++;
            if (dso_kf_count > 10)
            {
               if (dso_kf_count == 11)
                   timestamp_init = KF_timestamp;

               if (time_btwn_track_vec.empty())
                   time_btwn_track_vec.push_back(0);
               else
               {
                   double time_elapsed = KF_timestamp-timestamp_prev;
                   time_btwn_track_vec.push_back(time_elapsed);
               }

               timestamp_prev = KF_timestamp;

               if (time_btwn_track_vec.size() == 30)
               {
                   double KF_sec = (accumulate(time_btwn_track_vec.begin()+1, time_btwn_track_vec.end(), 0.0)/(time_btwn_track_vec.size()-1));
                   KF_rate = 1/KF_sec;
                   time_btwn_track_vec.erase(time_btwn_track_vec.begin());

                   std::cout << std::setprecision(3)<< "                                                        KF_rate = " << KF_rate << std::endl;
               }
            }

            orb_Tcw_final = mpSLAM->TrackMonocularDSO(
                                              KF_image,
                                              KF_timestamp,
                                              calib,
                                              Tcw_dso,
                                              T_inc_dso,
                                              scale_w_orb_from_w_dso,
                                              uvzvar_2send,
                                              depths_map_2send);


            if (orb_Tcw_final.empty()) continue;

            if (mpSLAM->mpTracker->mState == 3 && mpSLAM->mpMap->KeyFramesInMap() > 20)
            {
                lost_at_least_once = true;
                nKFs_in_map_when_lost = mpSLAM->mpMap->KeyFramesInMap();
            }

            std::vector<std::vector<cv::Mat>> LastGoodKeyframePositions_orb_dso =
                    mpSLAM->mpMap->GetLastGoodKeyframePositions_orb_dso(nPositions_for_scale,3);

            if (!LastGoodKeyframePositions_orb_dso.empty())
            {
                std::vector<cv::Mat> LastGoodKeyframePositions_orb = LastGoodKeyframePositions_orb_dso.at(0);
                std::vector<cv::Mat> LastGoodKeyframePositions_dso = LastGoodKeyframePositions_orb_dso.at(1);

                orb_positions = LastGoodKeyframePositions_orb.at(0);
                dso_positions = LastGoodKeyframePositions_dso.at(0);

                for (size_t i = 1; i < LastGoodKeyframePositions_orb.size(); i++)
                {
                    cv::hconcat(orb_positions, LastGoodKeyframePositions_orb.at(i), orb_positions);
                    cv::hconcat(dso_positions, LastGoodKeyframePositions_dso.at(i), dso_positions);
                }

                bool symmetric_scale = false;
                bool hold_rotation_and_scale = (lost_at_least_once && mpSLAM->mpMap->KeyFramesInMap() < nKFs_in_map_when_lost+30);

                ComputeSim3Alignment(dso_positions, orb_positions, Tcw_dso, orb_Tcw_final, symmetric_scale, hold_rotation_and_scale);

                std::cout << "scale = " << scale_w_orb_from_w_dso << std::endl;

                if (nPositions_for_scale < 30 )
                    nPositions_for_scale++;
            }


        }
        else if (DSO_full_stop_called)
        {
            wait_counter ++;
            if (wait_counter > 1 && wait_counter < 20)
                std::cout << "wait_counter = " << wait_counter << std::endl;

            if (wait_counter > 20 && !mpSLAM->mpLocalMapper->CheckNewKeyFrames() && mpSLAM->mpLocalMapper->AcceptKeyFrames())
            {
                // Stop all threads
                mpSLAM->Shutdown();
                // Save camera trajectory
                mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_seong.txt");
                ros::shutdown();
                break;
            }
        }

    }
}

void rosWrapper::GetStartEndTimstampsWithGroundTruth(const int &seq)
{
    switch (seq)
    {
        case 1: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461161071.4982504845, 1461161137.4567632675}); break;
        case 2: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461161178.6727609634, 1461161229.1699848175}); break;
        case 3: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461161328.9650213718, 1461161403.1232075691}); break;
        case 4: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461161520.9750673771, 1461161628.6895759106}); break;
        case 5: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461161726.6008017063, 1461161825.6758077145}); break;
        case 6: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461162063.8811976910, 1461162130.7378461361}); break;
        case 7: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461163015.4735877514, 1461163056.9093201160}); break;
        case 8: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461163291.6855993271, 1461163353.1624188423}); break;
        case 9: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461243320.7023031712, 1461243344.4011223316}); break;
        case 10: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461243988.0257508755, 1461244010.3456757069}); break;
        case 11: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461244107.4196059704, 1461244144.4949066639}); break;
        case 12: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461244288.8154542446, 1461244358.0770063400}); break;
        case 13: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461244440.8208558559, 1461244498.4001362324}); break;
        case 14: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461244592.8200676441, 1461244630.0412015915}); break;
        case 15: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461245028.4155094624, 1461245116.8547542095}); break;
        case 16: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1461245223.5159115791, 1461245267.8127353191}); break;
        case 17: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463141891.0046992302, 1463141997.0276727676}); break;
        case 18: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463142076.5940971375, 1463142203.2183961868}); break;
        case 19: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463142303.2247116566, 1463142481.3366863728}); break;
        case 20: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463142553.7236843109, 1463142663.8463418484}); break;
        case 21: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463142810.3226928711, 1463143045.8487863541}); break;
        case 22: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463143151.9068000317, 1463143438.2356007099}); break;
        case 23: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463162916.6485502720, 1463163014.1907718182}); break;
        case 24: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463163693.4866223335, 1463163783.0303802490}); break;
        case 25: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463163537.7056682110, 1463163653.5851418972}); break;
        case 26: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463224719.7863054276, 1463224784.9739828110}); break;
        case 27: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463227476.5903253555, 1463227566.7456212044}); break;
        case 28: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463225296.0284068584, 1463225446.4850461483}); break;
        case 29: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463225763.1026561260, 1463226000.3852863312}); break;
        case 30: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463226068.2597951889, 1463226128.2925992012}); break;
        case 31: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463226167.7454040051, 1463226291.7948756218}); break;
        case 32: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463226330.2510831356, 1463226431.5385615826}); break;
        case 33: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463226567.3644964695, 1463226631.5331330299}); break;
        case 34: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1463226757.9684526920, 1463226936.0751452446}); break;
        case 35: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464104388.1526153088, 1464104440.7128968239}); break;
        case 36: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464104617.0829122066, 1464104666.5441088676}); break;
        case 37: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464104833.0579493046, 1464104904.5163578987}); break;
        case 38: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464105904.5865836143, 1464105999.9566307068}); break;
        case 39: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464106077.5197823048, 1464106190.9717366695}); break;
        case 40: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464106968.9881682396, 1464107113.8804976940}); break;
        case 41: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464106270.0599508286, 1464106366.6330263615}); break;
        case 42: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464884544.5654733181, 1464884731.9463992119}); break;
        case 43: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464884848.1600911617, 1464884914.3044393063}); break;
        case 44: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464885062.1931023598, 1464885128.3405272961}); break;
        case 45: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464887324.7512311935, 1464887397.2393894196}); break;
        case 46: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464887563.4321048260, 1464887673.9320781231}); break;
        case 47: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464888258.0453054905, 1464888361.3473515511}); break;
        case 48: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464888380.4876818657, 1464888482.6377594471}); break;
        case 49: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464888821.9251492023, 1464888935.8885941505}); break;
        case 50: tsNoGroundTruth.insert(tsNoGroundTruth.begin(), {1464888988.9354631901, 1464889117.2241330147}); break;
    }







}


bool rosWrapper::DisableKeyframeCulling(const double &timestamp)
{
    if (timestamp < tsNoGroundTruth.front() || timestamp > tsNoGroundTruth.back())
        return true;

    return false;

}

void rosWrapper::ComputeSim3Alignment(
        const cv::Mat &dso_positions_,
        const cv::Mat &orb_positions_,
        const cv::Mat &latest_Tcw_dso_,
        const cv::Mat &latest_Tcw_orb_,
        const bool &symmetry_,
        const bool &hold_rotation_and_scale_)
{
    if (!hold_rotation_and_scale_)
    {
        // 1. Compute centroids:
        cv::Mat dso_centroid, orb_centroid;
        reduce(dso_positions_, dso_centroid, 1, CV_REDUCE_AVG);
        reduce(orb_positions_, orb_centroid, 1, CV_REDUCE_AVG);

        cv::Mat dso_pos_m_centroid = dso_positions_.clone();
        cv::Mat orb_pos_m_centroid = orb_positions_.clone();
        for (int i = 0; i < dso_pos_m_centroid.cols; i++)
            dso_pos_m_centroid.rowRange(0,3).col(i) = dso_pos_m_centroid.rowRange(0,3).col(i) - dso_centroid;
        for (int i = 0; i < orb_pos_m_centroid.cols; i++)
            orb_pos_m_centroid.rowRange(0,3).col(i)= orb_pos_m_centroid.rowRange(0,3).col(i) - orb_centroid;

         R_w_orb_from_w_dso = cv::Mat::eye(3,3,CV_32F);

        if (!symmetry_)
        {
            cv::Mat H = dso_pos_m_centroid * orb_pos_m_centroid.t();
            cv::Mat S,U,V;
            cv::SVD::compute(H,S,U,V);
            V = V.t();

            R_w_orb_from_w_dso = V*U.t();
            if (cv::determinant(R_w_orb_from_w_dso) < 0)
            {
                V.rowRange(0,3).col(2) *= -1;
                R_w_orb_from_w_dso = V*U.t();
            }
        }


        float den = 0;
        float num = 0;

        for (int i = 0; i<dso_pos_m_centroid.cols; i++)
        {
            cv::Mat num_inc;
            if (!symmetry_)
                num_inc = orb_pos_m_centroid.rowRange(0,3).col(i).t()*R_w_orb_from_w_dso*dso_pos_m_centroid.rowRange(0,3).col(i);
            else
                num_inc = orb_pos_m_centroid.rowRange(0,3).col(i).t()*orb_pos_m_centroid.rowRange(0,3).col(i);

            cv::Mat den_inc = dso_pos_m_centroid.rowRange(0,3).col(i).t()*dso_pos_m_centroid.rowRange(0,3).col(i);
            num += num_inc.at<float>(0,0);
            den += den_inc.at<float>(0,0);

        }

        if (!symmetry_)
            scale_w_orb_from_w_dso = num/den;
        else
            scale_w_orb_from_w_dso = sqrt(num/den);
    }

//    t_w_orb_from_w_dso = orb_centroid - scale_w_orb_from_w_dso*R_w_orb_from_w_dso*dso_centroid;

    cv::Mat latest_orb_position = -latest_Tcw_orb_.rowRange(0,3).colRange(0,3).t()*latest_Tcw_orb_.rowRange(0,3).col(3);
    cv::Mat latest_dso_position = -latest_Tcw_dso_.rowRange(0,3).colRange(0,3).t()*latest_Tcw_dso_.rowRange(0,3).col(3);
    t_w_orb_from_w_dso = latest_orb_position - scale_w_orb_from_w_dso*R_w_orb_from_w_dso*latest_dso_position;

    // Compute Sim(3) S_w_orb_from_w_dso:
    cv::Mat sR_w_orb_from_w_dso = scale_w_orb_from_w_dso*R_w_orb_from_w_dso;
    sR_w_orb_from_w_dso.copyTo(S_w_orb_from_w_dso.rowRange(0,3).colRange(0,3));
    t_w_orb_from_w_dso.copyTo(S_w_orb_from_w_dso.rowRange(0,3).col(3));

    // Compute Sim(3) S_w_dso_from_w_orb:
    cv::Mat sR_w_dso_from_w_orb = (1/scale_w_orb_from_w_dso)*R_w_orb_from_w_dso.t();
    cv::Mat t_w_dso_from_w_orb = -sR_w_dso_from_w_orb*t_w_orb_from_w_dso;
    sR_w_dso_from_w_orb.copyTo(S_w_dso_from_w_orb.rowRange(0,3).colRange(0,3));
    t_w_dso_from_w_orb.copyTo(S_w_dso_from_w_orb.rowRange(0,3).col(3));


    Sim3Aligned = true;
}


void rosWrapper::ComputeUVZVARandDepthsMapFromDSO(
        const std::vector<float> &calib_from_dso,
        const cv::Mat &Rcw_from_dso,
        const cv::Mat &tcw_from_dso,
        const std::vector<cv::Mat> &points_from_dso)
{
    uvzvar_2send.clear();
    std::vector<float> depths_map(img_width*img_height);
    std::vector<float> weightSums_map(img_width*img_height);

    int pix_thr = 2;

    for (auto point_from_dso : points_from_dso)
    {
        cv::Mat point_DSO_w =
                (cv::Mat_<float>(3,1) <<
                        point_from_dso.at<float>(0),
                        point_from_dso.at<float>(1),
                        point_from_dso.at<float>(2));

        cv::Mat point_DSO_c = Rcw_from_dso*point_DSO_w + tcw_from_dso;

        float x_DSO_c = point_DSO_c.at<float>(0);
        float y_DSO_c = point_DSO_c.at<float>(1);
        float z_DSO_c = point_DSO_c.at<float>(2);
        float idepth_var = point_from_dso.at<float>(3);
        float weight = sqrtf(1e-3 / (idepth_var+1e-12));

        // Check Positive depth:
        if (z_DSO_c < 0.0f)
        {
            continue;
        }
        // Project in image and check it is not out-of-bounds:
        float invz = 1.0f/z_DSO_c;
//        float fx = mpSLAM->mpTracker->mCurrentFrame.fx;
//        float fy = mpSLAM->mpTracker->mCurrentFrame.fy;
//        float cx = mpSLAM->mpTracker->mCurrentFrame.cx;
//        float cy = mpSLAM->mpTracker->mCurrentFrame.cy;
        float fx = calib_from_dso.at(0)*2;
        float fy = calib_from_dso.at(1)*2;
        float cx = calib_from_dso.at(2)*2+0.5;
        float cy = calib_from_dso.at(3)*2+0.5;
        float minX = mpSLAM->mpTracker->mCurrentFrame.mnMinX; // 0
        float maxX = mpSLAM->mpTracker->mCurrentFrame.mnMaxX; // 640
        float minY = mpSLAM->mpTracker->mCurrentFrame.mnMinY; // 0
        float maxY = mpSLAM->mpTracker->mCurrentFrame.mnMaxY; // 480


        int u = fx*x_DSO_c*invz+cx;
        int v = fy*y_DSO_c*invz+cy;

        //std::cout << "minX, maxX, minY, maxY = " << minX << ", " << maxX << ", " << minY << ", " << maxY << std::endl;

        if(u<minX || u>maxX-1 || v<minY || v>maxY-1)
        {
            continue;
        }

        cv::Mat uvzvar_member = (cv::Mat_<float>(4,1) << u, v, z_DSO_c, idepth_var);
        uvzvar_2send.push_back(uvzvar_member);

        int u_min_window = std::max(0, u-pix_thr);
        int u_max_window = std::min(img_width-1, u+pix_thr);
        int v_min_window = std::max(0, v-pix_thr);
        int v_max_window = std::min(img_height-1, v+pix_thr);

        for (int r=v_min_window; r<v_max_window; r++)
            for (int c=u_min_window; c<u_max_window; c++)
            {
                if ((r==v_min_window && (c==u_min_window || c==u_min_window+1)) ||
                    (r==v_min_window && (c==u_max_window || c==u_max_window-1)) ||
                    (r==v_max_window && (c==u_min_window || c==u_min_window+1)) ||
                    (r==v_max_window && (c==u_max_window || c==u_max_window-1)) ||
                    (r==v_min_window+1 && (c==u_min_window || c==u_max_window)) ||
                    (r==v_max_window-1 && (c==u_min_window || c==u_max_window)))
                    continue;
                else
                {
                    depths_map.at(r*img_width+c)+= invz*weight; // inverse depth
                    weightSums_map.at(r*img_width+c) += weight;
                }
            }
    }

    for (int i=0; i<img_width*img_height; i++)
    {
        if (depths_map.at(i) > 0 && weightSums_map.at(i) > 0)
        {
            depths_map.at(i) /= weightSums_map.at(i); // inverse-variance weighted average of inverse depths
            depths_map.at(i) = 1/depths_map.at(i); // depth
        }
        else
            depths_map.at(i) = 0;
    }

    std::transform(depths_map.begin(), depths_map.end(),
            depths_map.begin(),std::bind1st(std::multiplies<float>(),scale_w_orb_from_w_dso));

    depths_map_2send = depths_map;
}

void rosWrapper::Reset()
{
    DSO_reset_called = false;

    mpSLAM->Reset();

    total_sub_count = 0;

    temp_clear_signal_received = true;
    sync_clear_signal_received = true;

    scale_w_orb_from_w_dso = 1;
    dso_positions = cv::Mat();
    orb_positions = cv::Mat();


}




