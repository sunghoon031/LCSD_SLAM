#include <dso_ros/dso_node.h>
#include <dso_ros/ros_output_wrapper.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dso_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::MultiThreadedSpinner spinner(2);

    dso_ros::DsoNode node(nh, nh_private);
    node.setting_loop_rate = 15; //Hz

    std::thread DsoThread(&dso_ros::DsoNode::RunDSO, &node);
    std::thread publishThread (&dso_ros::DsoNode::PublishMarginalizedStuffFromDSO, &node, node.setting_loop_rate);
    spinner.spin();
    DsoThread.join();
    publishThread.join();


  return 0;
}

dso_ros::DsoNode::DsoNode(ros::NodeHandle &n, ros::NodeHandle &n_private)
  : nh(n)
  , nh_private(n_private)
  , image_file("")
  , calib_file("")
  , vignette_file("")
  , gamma_file("")
  , rectified_file("")
  , stats_file("")
  , initial_timestamp(0)
  , display_GUI_for_my_system(false)
  //, full_system(new dso::FullSystem())
  //, undistorter()
{
    initParams();

//    if (dataset == 1) //1=EuRoC MAV using rosbag, 2=TUM monoVO
//        image_sub = n.subscribe<sensor_msgs::Image>("image_raw", 10, boost::bind(&dso_ros::DsoNode::imageCb, this, _1));

    /*****************************************************/
    /*   Setting the mode:                               */
    /*   1 = Run Our System                              */
    /*   2 = Generate rectified Full Resolution Images   */
    /*   3 = Generate rectified Half Resolution Images   */
    /*****************************************************/

    if (mode == 1)
    {
        KF_ID_calib_pose_points_sub = n.subscribe(
              "KF_ID_calib_pose_points", 10, &dso_ros::DsoNode::KF_ID_calib_pose_points_callback, this);

        display_GUI_for_my_system_sub = n.subscribe(
                "display_GUI_for_my_system", 10, &dso_ros::DsoNode::Display_GUI_callback, this);

        marginalized_KF_time_image_calib_pose_points_pub = n.advertise<std_msgs::Float64MultiArray>("marginalized_KF_time_image_calib_pose_points", 10);

        reset_count_pub = n.advertise<std_msgs::Int64>("reset", 10);

        rectified_live_image_pub = n.advertise<sensor_msgs::Image>("image_rectified", 10);
    }
    else if (mode == 2 || mode ==3)
        rectified_live_image_pub = n.advertise<sensor_msgs::Image>("image_rectified", 10);

    dso::setting_desiredImmatureDensity = 600;
    dso::setting_desiredPointDensity = 800;
    dso::setting_minFrames = 4;
    dso::setting_maxFrames = 6;
    dso::setting_maxOptIterations = 4;
    dso::setting_minOptIterations = 1;
    dso::setting_logStuff = false;
    dso::setting_kfGlobalWeight = 1;

    dso::setting_pointLocalSmooth = false; // For the current version, keep it as false.
    dso::setting_patchRadius = 5; // For the current version, this is not used.

    dso::setting_onlyLogKFPoses = true;
    dso::setting_render_display3D = false;
    dso::setting_render_displayDepth = false;
    dso::setting_render_displayVideo = false;
    dso::setting_render_displayResidual = false;
    dso::setting_render_renderWindowFrames = false;
    dso::setting_render_plotTrackingFull = false;
    dso::setting_render_displayCoarseTrackingFull = false;

    original_setting_photometricCalibration = dso::setting_photometricCalibration;


    //reset();
}

dso_ros::DsoNode::~DsoNode()
{
}

/*
void dso_ros::DsoNode::imageCb(const sensor_msgs::ImageConstPtr &img)
{
    // this is needed to avoid initial freeze of whole algorithm. I don't know why
    // it needs couple initial frames to make reset function.
    if (frame_ID == 3) {
    reset();
    }
    cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    if (dso::setting_fullResetRequested) {
    ROS_ERROR_STREAM("[DSO_ROS]: System reset requested from DSO");
    reset();
    } else if (full_system->initFailed) {
    ROS_ERROR_STREAM("[DSO_ROS]: DSO init failed");
    reset();
    }

    dso::MinimalImageB min_img((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,
                             (unsigned char *)cv_ptr->image.data);

    double timestamp_double = cv_ptr->header.stamp.toSec();

    if (mode == 1) // My system
    {
        // Geometric Undistortion
        dso::setting_photometricCalibration = 0;
        std::shared_ptr<dso::ImageAndExposure> geo_undist_img(undistorter->undistort<unsigned char>(&min_img, 1, timestamp_double, 1.0f));
        // Add to system
        full_system->addActiveFrame(geo_undist_img.get(), frame_ID, half_resolution_dso);

        // Get the image height and width:
        image_h = geo_undist_img->h;
        image_w = geo_undist_img->w;
        image_h_half = geo_undist_img->h_half;
        image_w_half = geo_undist_img->w_half;


//        // Print result
//        full_system->printResult("/home/seonghunlee/ros/dso_ros_private/catkin_ws/src/dso_ros_private/KeyFrameTrajectory_DSO.txt");

        {
            std::unique_lock<std::mutex> lock(frame_sync_mtx);
            frame_ptrs_sync.push_back(geo_undist_img);
            frame_timestamps_sync.push_back(timestamp_double);
            frame_IDs_sync.push_back(frame_ID);
            frame_ID++;
        }
    }
    else if (mode == 2 || mode == 3) // original DSO
    {
        // Geometric (& Photometric) Undistortion
        std::shared_ptr<dso::ImageAndExposure> full_undist_img(undistorter->undistort<unsigned char>(&min_img, 1, timestamp_double, 1.0f));

        // Add to system
        full_system->addActiveFrame(full_undist_img.get(), frame_ID, half_resolution_dso);

        // Get the image height and width:
        image_h = full_undist_img->h;
        image_w = full_undist_img->w;
        image_h_half = full_undist_img->h_half;
        image_w_half = full_undist_img->w_half;


        // Print result
        if (half_resolution_dso)
            full_system->printResult("/home/seonghunlee/ros/dso_ros_private/catkin_ws/src/dso_ros_private/KeyFrameTrajectory_DSO_half.txt");
        else
            full_system->printResult("/home/seonghunlee/ros/dso_ros_private/catkin_ws/src/dso_ros_private/KeyFrameTrajectory_DSO_full.txt");
    }
    else // original ORB
    {
        // Geometric Undistortion
       dso::setting_photometricCalibration = 0;
       std::shared_ptr<dso::ImageAndExposure> geo_undist_img(undistorter->undistort<unsigned char>(&min_img, 1, timestamp_double, 1.0f));

       // Publish the rectified image:
       sensor_msgs::ImageConstPtr rectified_live_image_msg = ConvertImageAndExposure2ImagePtr(geo_undist_img, half_resolution_orb);
       rectified_live_image_pub.publish(rectified_live_image_msg);

    }
}
*/

void dso_ros::DsoNode::Display_GUI_callback(const std_msgs::Empty &msg)
{
    display_GUI_for_my_system = true;
}

void dso_ros::DsoNode::KF_ID_calib_pose_points_callback(const std_msgs::Float32MultiArray& id_calib_pose_points)
{
    std::vector<float> ID_calib_pose_points_received = id_calib_pose_points.data;

    // Get KF IDs
    int num_KFs = (int) ID_calib_pose_points_received.front();
    ID_calib_pose_points_received.erase(ID_calib_pose_points_received.begin());

    assert (num_KFs > 0);

    std::vector<int> KF_IDs_received;
    while (num_KFs > 0)
    {
        KF_IDs_received.push_back((int) ID_calib_pose_points_received.front());
        ID_calib_pose_points_received.erase(ID_calib_pose_points_received.begin());
        num_KFs--;
    }

    for (auto ID: KF_IDs_received)
    {
        auto it_KF_ID = std::find(
                KF_IDs_temp.begin(),
                KF_IDs_temp.end(),
                ID);
        if (it_KF_ID == KF_IDs_temp.end())
            KF_IDs_temp.push_back(ID);
    }

    // Get marginalized KF ID (or the most recent KF ID) and its calibration parameters and pose
    int marginalized_KF_ID = (int) ID_calib_pose_points_received.front();
    ID_calib_pose_points_received.erase(ID_calib_pose_points_received.begin());

    bool marginalized = (marginalized_KF_ID == -1 ? false : true);

    std::vector<float> calib(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+4);
    ID_calib_pose_points_received.erase(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+4);

    std::vector<float> pose(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+12);
    ID_calib_pose_points_received.erase(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+12);

    if (marginalized)
    {
        marginalized_KF_IDs_temp.push_back(marginalized_KF_ID);
        marginalized_KF_calib_temp.push_back(calib);
        marginalized_KF_poses_temp.push_back(pose);
    }

    // Get marginalized points
    std::vector<float> points = ID_calib_pose_points_received;
    ID_calib_pose_points_received.clear();
    for (auto p : points)
        marginalized_points_temp.push_back(p);

    if (marginalized)
    {
        marginalized_KF_points_temp.push_back(marginalized_points_temp);
        marginalized_points_temp.clear();
    }

}


void dso_ros::DsoNode::RunDSO()
{
    /*********************************************************/
    /*   Step 1: Initialize ImageFolderReader, FullSystem,   */
    /*           and outpur wrapper                          */
    /*********************************************************/

    reader = new ImageFolderReader(image_file,calib_file, gamma_file, vignette_file);
    reader->setGlobalCalibration(half_resolution);

    std::shared_ptr<dso::ImageAndExposure> img_temp(reader->getImage(0));
    image_h = img_temp.get()->h;
    image_w = img_temp.get()->w;
    image_h_half = image_h/2;
    image_w_half = image_w/2;

    fullSystem = new FullSystem();
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed==0);

    if (display_GUI)
    {
        if (half_resolution)
            fullSystem->outputWrapper.push_back(new dso::IOWrap::PangolinDSOViewer(image_w_half,image_h_half));
        else
            fullSystem->outputWrapper.push_back(new dso::IOWrap::PangolinDSOViewer(image_w,image_h));
    }

    if (mode==1)
        fullSystem->outputWrapper.push_back(new dso_ros::ROSOutputWrapper(nh, nh_private));

    /***************************************/
    /*   Step 2: Get IDs and timestamps    */
    /***************************************/

    std::vector<int> idsToPlay;
    std::vector<double> timesToPlayAt;

    int lstart=0;
    int lend=100000;
    int linc = 1;

    for(int i=lstart;i>= 0 && i< reader->getNumImages() && linc*i < linc*lend;i+=linc)
    {
       idsToPlay.push_back(i);
       if(timesToPlayAt.size() == 0)
       {
           timesToPlayAt.push_back((double)0);
       }
       else
       {
           double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
           double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
           timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
       }
    }

    double sleep_init = (timesToPlayAt.at(1)-timesToPlayAt.at(0))*1e6;

    /*******************************/
    /*   Step 3: Preload images    */
    /*******************************/

    std::vector<std::shared_ptr<dso::ImageAndExposure> > preloadedImages;
    std::vector<std::shared_ptr<dso::ImageAndExposure> > preloadedImages_noPhotoCalib;
    std::vector<std::shared_ptr<dso::MinimalImageB> > preloadedRawImages;

    if (preRectification || mode==2 || mode==3)
    {
        if (dataset==2 && mode==1)
            printf("LOADING AND RECTIFYING (AND PHOTOMETRICALLY CALIBRATING IF APPLICABLE) ALL IMAGES!\n");
        else
            printf("LOADING AND RECTIFYING ALL IMAGES (NO PHOTOMETRIC CALIBRATION)!\n");

        for(int ii=0;ii<(int)idsToPlay.size(); ii++)
        {
           int i = idsToPlay[ii];
           std::shared_ptr<dso::ImageAndExposure> img(reader->getImage(i));
           preloadedImages.emplace_back(img);
        }

        // In our system, we use images wihtout photo calibration for ORB-SLAM
        if (dataset==2 && mode == 1 )
        {
            dso::setting_photometricCalibration = 0;

            for(int ii=0;ii<(int)idsToPlay.size(); ii++)
            {
               int i = idsToPlay[ii];
               std::shared_ptr<dso::ImageAndExposure> img(reader->getImage(i));
               preloadedImages_noPhotoCalib.emplace_back(img);
            }

            dso::setting_photometricCalibration = original_setting_photometricCalibration;

        }

        printf("LOADING COMPLETE!\n");
    }
    else // (!preRectification && mode == 1)
    {
        printf("LOADIDNG ALL RAW IMAGES (NO GEOMETRIC OR PHOTOMETRIC CALIB)!\n");
        for(int ii=0;ii<(int)idsToPlay.size(); ii++)
        {
            int i = idsToPlay[ii];
            std::shared_ptr<dso::MinimalImageB> img(reader->getImageRaw(i));
            preloadedRawImages.push_back(img);
        }
        printf("LOADING COMPLETE!\n");
    }

    /***********************************************/
    /*   Step 4: Generate rectified images         */
    /*           (ONLY IF this mode is selected)   */
    /***********************************************/

    if (mode==2 || mode==3)
    {
        for(int ii=0;ii<(int)idsToPlay.size(); ii++)
        {
            // Publish the rectified image:
            sensor_msgs::ImageConstPtr rectified_live_image_msg = ConvertImageAndExposure2ImagePtr(preloadedImages[ii], half_resolution);
            rectified_live_image_pub.publish(rectified_live_image_msg);
            std::cout << "Publish " << ii << std::endl;
            usleep(10000);

            std::string image_filename(rectified_file);

            std::ostringstream s1;
            s1 << std::setw(2) << std::setfill('0') << sequence;
            image_filename+=s1.str();

            image_filename+="/images_rectifiedFullResolution/";

            std::ostringstream s2;
            s2 << std::setw(4) << std::setfill('0') << ii;
            image_filename+=s2.str();

            image_filename+=".jpg";

            std::ifstream f1(image_filename.c_str());

            bool frame_missed = false;
            if( !f1.good() )
            {
                std::cout << "Frame not saved. Wait..." << std::endl;
                frame_missed = true;
                usleep(1000000);
            }

            if (frame_missed)
            {
                std::ifstream f2(image_filename.c_str());
                if (!f2.good())
                {
                    std::cout << "Frame missed. Repeat!" << std::endl;
                    ii--;
                }
                else
                    std::cout << "Saved it after waiting a bit! ;)" << std::endl;

            }
        }

        preloadedImages.clear();
        delete reader;

        exit(1);
        return;
    }

    /*******************************/
    /*   Step 5: Run our system!   */
    /*******************************/

    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    clock_t started = clock();
    double sInitializerOffset=0;

    nSkippedFrames = 0;
    nTotalFramesAfterInit = 0;
    trackingStartTimestamp = 0;
    trackingEndTimestamp = 0;
    startTimestamp = reader->getTimestamp(idsToPlay.front());
    endTimestamp = reader->getTimestamp(idsToPlay.back());

    double resetAllowedTimeFactor = 0.3;
    double timestampThreshold = timesToPlayAt.at((int) timesToPlayAt.size()*resetAllowedTimeFactor);


    while (!idsToPlay.empty())
    {
        if (display_GUI_for_my_system)
        {
            sensor_msgs::ImageConstPtr rectified_live_image_msg;
            if (preRectification)
                rectified_live_image_msg = ConvertImageAndExposure2ImagePtr(preloadedImages_noPhotoCalib.front(), false);
            else
            {
                std::shared_ptr<dso::ImageAndExposure> img_noPhotoCalib(reader->getImageFromImageRaw(preloadedRawImages.front().get(), idsToPlay.front()));
                rectified_live_image_msg = ConvertImageAndExposure2ImagePtr(img_noPhotoCalib, false);
            }
            rectified_live_image_pub.publish(rectified_live_image_msg);
        }

        if(!fullSystem->initialized)    // if not initialized: reset start time.
        {
            usleep((int)(sleep_init));
            gettimeofday(&tv_start, NULL);
            started = clock();
            sInitializerOffset = timesToPlayAt[idsToPlay.front()];

        }

        if (fullSystem->initFailed || dso::setting_fullResetRequested)
            reset();

        bool skipFrame=false;

        struct timeval tv_now; gettimeofday(&tv_now, NULL);
        double sSinceStart = sInitializerOffset + ((tv_now.tv_sec-tv_start.tv_sec) + (tv_now.tv_usec-tv_start.tv_usec)/(1000.0f*1000.0f));


        if (fullSystem->initialized)
        {
            if(sSinceStart < timesToPlayAt[idsToPlay.front()])
            {
                usleep((int)((timesToPlayAt[idsToPlay.front()]-sSinceStart)*1000*1000));
            }
            else if(sSinceStart > timesToPlayAt[idsToPlay.front()]/*+0.5+0.1*(ii%2)*/)
            {
                skipFrame=true;
                nSkippedFrames++;

                //printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
                double overdue_ms = (sSinceStart-timesToPlayAt[idsToPlay.front()])*1000;
                std::cout << std::setprecision(15) << "SKIP FRAME (#"<< nSkippedFrames <<")! "<< overdue_ms << "ms overdue "<< std::endl;

            }
        }

        if(!skipFrame)
        {
            if (fullSystem->initialized)
                nTotalFramesAfterInit++;

            struct timeval tv_beforeTracking, tv_afterTracking;

            if (preRectification)
            {

                {
                    std::unique_lock<std::mutex> lock(frame_sync_mtx);
                    // Send images without photo calib to ORB-SLAM
                    if (dataset==1)
                        frame_ptrs_sync.push_back(preloadedImages.front());
                    else if (dataset==2)
                        frame_ptrs_sync.push_back(preloadedImages_noPhotoCalib.front());

                    frame_timestamps_sync.push_back(reader->getTimestamp(idsToPlay.front()));
                    frame_IDs_sync.push_back(idsToPlay.front());
                }

                gettimeofday(&tv_beforeTracking, NULL);
                // Use images with photo calib in DSO
                fullSystem->addActiveFrame(preloadedImages.front().get(), idsToPlay.front(), half_resolution);
                gettimeofday(&tv_afterTracking, NULL);
            }
            else
            {
                // Send images without photo calib to ORB-SLAM
                dso::setting_photometricCalibration = 0;
                std::shared_ptr<dso::ImageAndExposure> img_noPhotoCalib(reader->getImageFromImageRaw(preloadedRawImages.front().get(), idsToPlay.front()));
                dso::setting_photometricCalibration = original_setting_photometricCalibration;

                {
                    std::unique_lock<std::mutex> lock(frame_sync_mtx);
                    frame_ptrs_sync.push_back(img_noPhotoCalib);
                    frame_timestamps_sync.push_back(reader->getTimestamp(idsToPlay.front()));
                    frame_IDs_sync.push_back(idsToPlay.front());
                }

                gettimeofday(&tv_beforeTracking, NULL);
                // Use images with photo calib in DSO
                std::shared_ptr<dso::ImageAndExposure> img(reader->getImageFromImageRaw(preloadedRawImages.front().get(), idsToPlay.front()));
                fullSystem->addActiveFrame(img.get(), idsToPlay.front(), half_resolution);
                gettimeofday(&tv_afterTracking, NULL);
            }

            double trackingSecond = (tv_afterTracking.tv_sec-tv_beforeTracking.tv_sec) + (tv_afterTracking.tv_usec-tv_beforeTracking.tv_usec)/(1000.0f*1000.0f);
            double trackingMillisecond = trackingSecond*1000;
            trackingTimes.push_back(trackingMillisecond);
        }

        if(fullSystem->isLost)
        {
            if (sSinceStart < timestampThreshold)
            {
                std::cout << "Lost within first 30%. Reset!" << std::endl;
                dso::setting_fullResetRequested = true;
            }
            else
            {
                printf("LOST!!\n");
                break;
            }
        }
        else if (fullSystem->initialized)
        {
            trackingEndTimestamp = reader->getTimestamp(idsToPlay.front());
            if (trackingStartTimestamp == 0)
                trackingStartTimestamp = reader->getTimestamp(idsToPlay.front());
        }

        // Don't forget to erase!
        if (preRectification)
        {
            preloadedImages.erase(preloadedImages.begin());
            preloadedImages_noPhotoCalib.erase(preloadedImages_noPhotoCalib.begin());
        }
        else
            preloadedRawImages.erase(preloadedRawImages.begin());

        idsToPlay.erase(idsToPlay.begin());
    }

    printf("TRACKING FINISHED!! \n");
    fullSystem->blockUntilMappingIsFinished();

    printf("Marginalize Everything By Force!! \n");
    for (int i = 0; i < 10; i++)
    {
        usleep(0.1*1000*1000);
        fullSystem->marginalizeEverythingByForce();
    }

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
        ow->publishFullStop();

    preloadedImages.clear();
    preloadedImages_noPhotoCalib.clear();
    preloadedRawImages.clear();

    /***************************************/
    /*   Step 6: Print timing statistics   */
    /***************************************/

    double trackingTimeMed, trackingTimeAvg, trackingTimeStd;
    if (trackingTimes.empty())
    {
        trackingTimeMed = 0;
        trackingTimeAvg = 0;
        trackingTimeStd = 0;
    }
    else
    {
        sort(trackingTimes.begin(), trackingTimes.end());
        trackingTimeMed = trackingTimes[trackingTimes.size()/2];
        trackingTimeAvg = accumulate(trackingTimes.begin(), trackingTimes.end(), 0.0)/trackingTimes.size();
        std::vector<double> diff(trackingTimes.size());
        std::transform(trackingTimes.begin(), trackingTimes.end(), diff.begin(), std::bind2nd(std::minus<double>(), trackingTimeAvg));
        trackingTimeStd = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)/trackingTimes.size();
        trackingTimeStd = std::sqrt(trackingTimeStd);
    }

    // Print result
    printf("PRINT RESULTS start!! \n");
    fullSystem->printResultSeong(
            stats_file,
            nSkippedFrames,
            nTotalFramesAfterInit,
            trackingTimeMed,
            trackingTimeAvg,
            trackingTimeStd,
            trackingStartTimestamp,
            trackingEndTimestamp,
            startTimestamp,
            endTimestamp);
    printf("PRINT RESULTS end!! \n");


    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }


    for (dso::IOWrap::Output3DWrapper *ow : fullSystem->outputWrapper) {
      ow->join();
      delete ow;
    }

    delete fullSystem;
    delete reader;

}

void dso_ros::DsoNode::PublishMarginalizedStuffFromDSO(const float &publish_loop_rate)
{
    if (mode != 1)
        return;

    ros::Rate loop_rate(publish_loop_rate);
    int wait_counter = 0;

    while (ros::ok())
    {
        loop_rate.sleep();

        if (KF_IDs_temp.size() == 0) continue;


        bool new_KF_added = false;
        for (auto KF_ID : KF_IDs_temp)
        {
            if (KF_IDs_sync.empty())
            {
                KF_IDs_sync.push_back(KF_IDs_temp.back());
                new_KF_added = true;
                break;
            }
            else
            {
                auto it_KF_ID = std::find(
                                KF_IDs_sync.begin(),
                                KF_IDs_sync.end(),
                                KF_ID);
                if (it_KF_ID == KF_IDs_sync.end() && KF_ID > KF_IDs_sync.back())
                {
                    KF_IDs_sync.push_back(KF_ID);
                    new_KF_added = true;
                    break;
                }
            }
        }

        if (KF_IDs_sync.empty())
            continue;

        assert(KF_IDs_sync.back() > 0);

        {
            std::unique_lock<std::mutex> lock(frame_sync_mtx);

            if (frame_IDs_sync.size() != frame_ptrs_sync.size()
                    || frame_IDs_sync.size() != frame_timestamps_sync.size())
            {
                std::cout << "Error: frame_sync vectors size mismatch !!!" << std::endl;
                std::cout << "frame_IDs_sync.size() = " << frame_IDs_sync.size()  <<std::endl;
                std::cout << "frame_ptrs_sync.size() = " << frame_ptrs_sync.size()  <<std::endl;
                std::cout << "frame_timestamps_sync.size() = " << frame_timestamps_sync.size()  <<std::endl;
                continue;
            }

            assert(frame_IDs_sync.size() == frame_ptrs_sync.size());
            assert(frame_IDs_sync.size() == frame_timestamps_sync.size());
        }



        if (new_KF_added)
        {
            std::unique_lock<std::mutex> lock(frame_sync_mtx);

            auto it_frame_IDs_history = std::find(
                            frame_IDs_sync.begin(),
                            frame_IDs_sync.end(),
                            KF_IDs_sync.back());

            if (it_frame_IDs_history != frame_IDs_sync.end())
            {
                auto it_frame_images = frame_ptrs_sync.begin();
                advance(it_frame_images, std::distance(frame_IDs_sync.begin(), it_frame_IDs_history));
                KF_ptrs_sync.push_back(*it_frame_images);


                auto it_frame_timestamps = frame_timestamps_sync.begin();
                advance(it_frame_timestamps, std::distance(frame_IDs_sync.begin(), it_frame_IDs_history));
                KF_timestamps_sync.push_back(*it_frame_timestamps);

                // Erase frame IDs, timestamps and image ptrs that are older than the KF ID:
                frame_IDs_sync.erase(frame_IDs_sync.begin(), it_frame_IDs_history);
                frame_ptrs_sync.erase(frame_ptrs_sync.begin(), it_frame_images);

                frame_timestamps_sync.erase(frame_timestamps_sync.begin(), it_frame_timestamps);
            }
            else
            {
                std::cout << "Error: KF ID does not exist in frame ID list !!!" << std::endl;
                std::cout << "KF_IDs_sync.back() = " << KF_IDs_sync.back() << std::endl;
                std::cout << "frame_IDs_sync.front() = " << frame_IDs_sync.front() << std::endl;
                std::cout << "frame_IDs_sync.back() = " << frame_IDs_sync.back() << std::endl;
                assert(it_frame_IDs_history != frame_IDs_sync.end());
            }
        }


        if (!marginalized_KF_IDs_temp.empty()
                && !marginalized_KF_poses_temp.empty()
                && !marginalized_KF_calib_temp.empty()
                && !marginalized_KF_points_temp.empty())
        {
            marginalized_KF_IDs_sync.push_back(marginalized_KF_IDs_temp.front());
            marginalized_KF_IDs_temp.erase(marginalized_KF_IDs_temp.begin());

            marginalized_KF_calib_sync.push_back(marginalized_KF_calib_temp.front());
            marginalized_KF_calib_temp.erase(marginalized_KF_calib_temp.begin());

            marginalized_KF_poses_sync.push_back(marginalized_KF_poses_temp.front());
            marginalized_KF_poses_temp.erase(marginalized_KF_poses_temp.begin());

            marginalized_KF_points_sync.push_back(marginalized_KF_points_temp.front());
            marginalized_KF_points_temp.erase(marginalized_KF_points_temp.begin());
        }

//        assert(KF_IDs_sync.size() == KF_timestamps_sync.size());
//        assert(KF_IDs_sync.size() == KF_ptrs_sync.size());

        auto it_marginalized_ID = std::find(
                        marginalized_KF_IDs_sync.begin(),
                        marginalized_KF_IDs_sync.end(),
                        KF_IDs_sync.front());

        if (it_marginalized_ID != marginalized_KF_IDs_sync.end())
        {
            marginalized_KF_ID = KF_IDs_sync.front();

            double marginalized_timestamp = KF_timestamps_sync.front();
            std::vector<float> marginalized_img;

            std::vector<float> marginalized_img_full(KF_ptrs_sync.front()->image,
                            KF_ptrs_sync.front()->image + image_h*image_w);
            marginalized_img = marginalized_img_full;

            if ( marginalized_img.size() != image_h*image_w)
                assert(marginalized_img.size() == image_h*image_w);

            // Since we cannot use the same iterator for different vector:
            auto iterator_calib = marginalized_KF_calib_sync.begin();
            advance(iterator_calib, std::distance(marginalized_KF_IDs_sync.begin(), it_marginalized_ID));
            std::vector<float> marginalized_calib = *iterator_calib;

            auto iterator_pose = marginalized_KF_poses_sync.begin();
            advance(iterator_pose, std::distance(marginalized_KF_IDs_sync.begin(), it_marginalized_ID));
            std::vector<float> marginalized_pose = *iterator_pose;

            auto iterator_points = marginalized_KF_points_sync.begin();
            advance(iterator_points, std::distance(marginalized_KF_IDs_sync.begin(), it_marginalized_ID));
            std::vector<float> marginalized_points = *iterator_points;

            // Publish the marginalized keyframe info:
            // 1st element: timestamp
            // 2nd & 3rd element: h & w
            // Next elements: image
            // Next elements: calibration (fx,fy,cx,cy)
            // Next elements: pose
            // Next elements: points (x,y,z,idepth_var_relative)
            std_msgs::Float64MultiArray marginalized_KF_time_image_calib_pose_points_msg;
            marginalized_KF_time_image_calib_pose_points_msg.data.push_back(marginalized_timestamp);

            marginalized_KF_time_image_calib_pose_points_msg.data.push_back(image_h);
            marginalized_KF_time_image_calib_pose_points_msg.data.push_back(image_w);
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                    marginalized_img.begin(),
                    marginalized_img.end());
            assert(marginalized_KF_time_image_calib_pose_points_msg.data.size() == image_h*image_w + 3);
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                                marginalized_calib.begin(),
                                marginalized_calib.end());
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                    marginalized_pose.begin(),
                    marginalized_pose.end());
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                    marginalized_points.begin(),
                    marginalized_points.end());
            marginalized_KF_time_image_calib_pose_points_pub.publish(marginalized_KF_time_image_calib_pose_points_msg);

            // Erase:
            KF_IDs_sync.erase(KF_IDs_sync.begin());
            KF_timestamps_sync.erase(KF_timestamps_sync.begin());
            KF_ptrs_sync.erase(KF_ptrs_sync.begin());
            marginalized_KF_IDs_sync.erase(it_marginalized_ID);
            marginalized_KF_calib_sync.erase(iterator_calib);
            marginalized_KF_poses_sync.erase(iterator_pose);
            marginalized_KF_points_sync.erase(iterator_points);

            wait_counter = 0;
        }
    }
}


void dso_ros::DsoNode::initParams()
{
    bool debug = nh_private.param<bool>("debug", false);
    if (debug) {
     dso::setting_debugout_runquiet = false;
     dso::setting_logStuff = true;
    } else {
     dso::setting_debugout_runquiet = true;
     dso::setting_logStuff = false;
    }

    nh_private.param<bool>("display_GUI", display_GUI, false);
    nh_private.param<double>("playback_speed", playbackSpeed, 1);
    nh_private.param<std::string>("image_file_path", image_file, "");
    nh_private.param<std::string>("calib_file_path", calib_file, "");
    nh_private.param<std::string>("vignette_file_path", vignette_file, "");
    nh_private.param<std::string>("gamma_file_path", gamma_file, "");
    nh_private.param<std::string>("rectified_file_path", rectified_file, "");
    nh_private.param<std::string>("stats_file_path", stats_file, "");
    nh_private.param<int>("mode", mode, 1);
    nh_private.param<int>("dataset", dataset, 1);
    nh_private.param<int>("sequence", sequence, 1);

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "     mode = " << mode << std::endl;

    dso::setting_semiDirect = false;
    half_resolution = false;
    preRectification = false;

    if (mode == 1)
    {
       std::cout << "     Run my system" << std::endl;
       dso::setting_semiDirect = true;
       half_resolution = true;
    }
    else if (mode == 2)
    {
       std::cout << "     Gnerate Rectified full Resolution Images, sequence = " << sequence <<std::endl;
    }
    else if (mode == 3)
    {
       std::cout << "     Gnerate Rectified half Resolution Images, sequence = " << sequence << std::endl;
       half_resolution = true;
    }

    if (dataset == 2 && mode == 1)
    {
       std::cout << "     Do Photometric Calibration !" << std::endl;
       dso::setting_photometricCalibration = 2;
    }
    else
    {
       std::cout << "     Don't Do Photometric Calibration !" << std::endl;
       dso::setting_photometricCalibration = 0;
       dso::setting_affineOptModeA = 0;
       dso::setting_affineOptModeB = 0;
    }

    std::cout << "     playback speed = x" << playbackSpeed << std::endl;

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


}

sensor_msgs::ImageConstPtr dso_ros::DsoNode::ConvertImageAndExposure2ImagePtr(std::shared_ptr<dso::ImageAndExposure> input, bool half_resolution)
{
    cv::Mat image_cv;

    if (half_resolution)
        image_cv = cv::Mat(input->h_half, input->w_half, CV_32FC1, input->image_half)*(1/254.0f);
    else
        image_cv = cv::Mat(input->h, input->w, CV_32FC1, input->image)*(1/254.0f);

    image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
    std_msgs::Header header;

    header.stamp = ros::Time(input->timestamp);
    cv_bridge::CvImage bridge_img(header, "mono8", image_cv);



    return bridge_img.toImageMsg();
}


void dso_ros::DsoNode::reset()
{
    printf("RESETTING!\n");
    setting_fullResetRequested=false;

    std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
    delete fullSystem;

    for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

    fullSystem = new FullSystem();
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed==0);

    fullSystem->outputWrapper = wraps;

    nSkippedFrames = 0;
    nTotalFramesAfterInit = 0;
    trackingStartTimestamp = 0;
    trackingTimes.clear();

    KF_IDs_temp.clear();

    KF_IDs_sync.clear();
    KF_timestamps_sync.clear();
    KF_ptrs_sync.clear();

    marginalized_KF_IDs_temp.clear();
    marginalized_KF_IDs_sync.clear();

    marginalized_KF_points_temp.clear();
    marginalized_KF_points_sync.clear();
    marginalized_points_temp.clear();

    marginalized_KF_calib_temp.clear();
    marginalized_KF_calib_sync.clear();

    marginalized_KF_poses_temp.clear();
    marginalized_KF_poses_sync.clear();

    reset_count_msg.data ++;
    reset_count_pub.publish(reset_count_msg);

    total_pub_count = 0;

    {
        std::unique_lock<std::mutex> lock(frame_sync_mtx);
        frame_IDs_sync.clear();
        frame_timestamps_sync.clear();
        frame_ptrs_sync.clear();
    }

}



