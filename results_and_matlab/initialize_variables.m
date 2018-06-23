ATEs_our_VO= []; ATEs_our_SLAM = []; 
nKeyframes_our_VO = []; nKeyframes_our_SLAM = []; 
nMapPoints_our_VO =[];  nMapPoints_our_SLAM =[]; 
nSkippedFrames_our_VO = [];  nSkippedFrames_our_SLAM = []; 
nFramesAfterInit_our_VO = [];  nFramesAfterInit_our_SLAM = []; 
trackingTimeMed_our_VO=[];  trackingTimeMed_our_SLAM=[]; 
trackingTimeAvg_our_VO=[];  trackingTimeAvg_our_SLAM=[]; 
trackingTimeStd_our_VO=[];  trackingTimeStd_our_SLAM=[]; 
nLoopClosures_our_VO=[];  nLoopClosures_our_SLAM=[]; 

ATEs_orb_VO_full = []; ATEs_orb_VO_half = []; ATEs_orb_SLAM_full = []; ATEs_orb_SLAM_half = [];
nKeyframes_orb_VO_full = []; nKeyframes_orb_VO_half = []; nKeyframes_orb_SLAM_full = []; nKeyframes_orb_SLAM_half = [];
nMapPoints_orb_VO_full =[]; nMapPoints_orb_VO_half =[]; nMapPoints_orb_SLAM_full =[]; nMapPoints_orb_SLAM_half =[];
nSkippedFrames_orb_VO_full = []; nSkippedFrames_orb_VO_half = []; nSkippedFrames_orb_SLAM_full = []; nSkippedFrames_orb_SLAM_half = [];
nFramesAfterInit_orb_VO_full = []; nFramesAfterInit_orb_VO_half = []; nFramesAfterInit_orb_SLAM_full = []; nFramesAfterInit_orb_SLAM_half = [];
trackingTimeMed_orb_VO_full=[]; trackingTimeMed_orb_VO_half=[]; trackingTimeMed_orb_SLAM_full=[]; trackingTimeMed_orb_SLAM_half=[];
trackingTimeAvg_orb_VO_full=[]; trackingTimeAvg_orb_VO_half=[]; trackingTimeAvg_orb_SLAM_full=[]; trackingTimeAvg_orb_SLAM_half=[];
trackingTimeStd_orb_VO_full=[]; trackingTimeStd_orb_VO_half=[]; trackingTimeStd_orb_SLAM_full=[]; trackingTimeStd_orb_SLAM_half=[];
nLoopClosures_orb_VO_full=[]; nLoopClosures_orb_VO_half=[]; nLoopClosures_orb_SLAM_full=[]; nLoopClosures_orb_SLAM_half=[];

ATEs_dso_default_full = []; ATEs_dso_default_half = []; ATEs_dso_reduced_full = []; ATEs_dso_reduced_half = [];
nKeyframes_dso_default_full = []; nKeyframes_dso_default_half = []; nKeyframes_dso_reduced_full = []; nKeyframes_dso_reduced_half = [];
nMapPoints_dso_default_full =[]; nMapPoints_dso_default_half =[]; nMapPoints_dso_reduced_full =[]; nMapPoints_dso_reduced_half =[];
nSkippedFrames_dso_default_full = []; nSkippedFrames_dso_default_half = []; nSkippedFrames_dso_reduced_full = []; nSkippedFrames_dso_reduced_half = [];
nFramesAfterInit_dso_default_full = []; nFramesAfterInit_dso_default_half = []; nFramesAfterInit_dso_reduced_full = []; nFramesAfterInit_dso_reduced_half = [];
trackingTimeMed_dso_default_full=[]; trackingTimeMed_dso_default_half=[]; trackingTimeMed_dso_reduced_full=[]; trackingTimeMed_dso_reduced_half=[];
trackingTimeAvg_dso_default_full=[]; trackingTimeAvg_dso_default_half=[]; trackingTimeAvg_dso_reduced_full=[]; trackingTimeAvg_dso_reduced_half=[];
trackingTimeStd_dso_default_full=[]; trackingTimeStd_dso_default_half=[]; trackingTimeStd_dso_reduced_full=[]; trackingTimeStd_dso_reduced_half=[];
nLoopClosures_dso_default_full=[]; nLoopClosures_dso_default_half=[]; nLoopClosures_dso_reduced_full=[]; nLoopClosures_dso_reduced_half=[];

format short

euroc_sequences = {...
        '_m1L-';'_m1R-';'_m2L-';'_m2R-';'_m3L-';'_m3R-';...
        '_m4L-';'_m4R-';'_m5L-';'_m5R-';'_v11L-';'_v11R-';...
        '_v12L-';'_v12R-';'_v13L-';'_v13R-';'_v21L-';'_v21R-';...
        '_v22L-';'_v22R-';'_v23L-';'_v23R-'};
    
gt_files = {...
        'EuRoC_MH_01_easy.csv';...
        'EuRoC_MH_02_easy.csv';...
        'EuRoC_MH_03_medium.csv';...
        'EuRoC_MH_04_difficult.csv';...
        'EuRoC_MH_05_difficult.csv';...
        'EuRoC_V1_01_easy.csv';...
        'EuRoC_V1_02_medium.csv';...
        'EuRoC_V1_03_difficult.csv';...
        'EuRoC_V2_01_easy.csv';...
        'EuRoC_V2_02_medium.csv';...
        'EuRoC_V2_03_difficult.csv'};