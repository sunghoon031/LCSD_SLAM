%% Initialize

clear all; close all; clc;
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% EuRoC MAV results

% [1] Set paths:
gt_dir = '~/LSV-SLAM/results_and_matlab/gtFiles/';
our_vo_dir = '~/LSV-SLAM/results_and_matlab/results/EuRoC_MAV/results_our_vo/';
our_slam_dir = '~/LSV-SLAM/results_and_matlab/results/EuRoC_MAV/results_our_slam/';
dso_default_dir = '~/LSV-SLAM/results_and_matlab/results/EuRoC_MAV/results_dso_default/';
dso_reduced_dir = '~/LSV-SLAM/results_and_matlab/results/EuRoC_MAV/results_dso_reduced/';
orb_vo_dir = '~/LSV-SLAM/results_and_matlab/results/EuRoC_MAV/results_orb_vo/';
orb_slam_dir = '~/LSV-SLAM/results_and_matlab/results/EuRoC_MAV/results_orb_slam/';

% [2] Select mode:
% - 1: Plot pre-evaluated results to make the same figures in the paper.
% - 2: Process your own results and plot.
% - 3: Plot your processed results (if you've already run with mode=2).
mode_EuRoC_MAV = 3;

% [3] If mode = 2, Select systems to evaluate:
% - 1: Our VO 
% - 2: Our SLAM
% - 3: DSO default
% - 4: DSO reduced 
% - 5: ORB-VO
% - 6: ORB-SLAM
systems_to_evaluate_EuRoC_MAV = [1,2,3,4,5,6];

% [4] Run !!
switch mode_EuRoC_MAV
    case 1
        disp('[EuRoC MAV] Mode 1: Plotting Pre-Evaluated Results...')
        load PRE_EVALUATED_RESULTS.mat;    
        PlotResults_EuRoC_MAV;
    
    case 2
        disp('[EuRoC MAV] Mode 2: Process and Plot...')
        ProcessResults_EuRoC_MAV;
        load OUR_VO_RESULTS_EuRoC_MAV.mat;
        load OUR_SLAM_RESULTS_EuRoC_MAV.mat;
        load DSO_DEFAULT_RESULTS_EuRoC_MAV.mat;
        load DSO_REDUCED_RESULTS_EuRoC_MAV.mat;
        load ORB_VO_RESULTS_EuRoC_MAV.mat;
        load ORB_SLAM_RESULTS_EuRoC_MAV.mat;      
        PlotResults_EuRoC_MAV;
    case 3
        disp('[EuRoC MAV] Mode 3: Plotting Your Pre-Processed Results...')
        load OUR_VO_RESULTS_EuRoC_MAV.mat;
        load OUR_SLAM_RESULTS_EuRoC_MAV.mat;
        load DSO_DEFAULT_RESULTS_EuRoC_MAV.mat;
        load DSO_REDUCED_RESULTS_EuRoC_MAV.mat;
        load ORB_VO_RESULTS_EuRoC_MAV.mat;
        load ORB_SLAM_RESULTS_EuRoC_MAV.mat;   
        PlotResults_EuRoC_MAV;
end



%% TUM monoVO results

% [1] Set paths
FILEPATH_gt = gt_dir;
FILEPATH_our_vo= '~/LSV-SLAM/results_and_matlab/results/TUM_monoVO/results_our_vo/';
FILEPATH_our_slam= '~/LSV-SLAM/results_and_matlab/results/TUM_monoVO/results_our_slam/';
FILEPATH_dso_default = '~/LSV-SLAM/results_and_matlab/results/TUM_monoVO/results_dso_default/';
FILEPATH_dso_reduced = '~/LSV-SLAM/results_and_matlab/results/TUM_monoVO/results_dso_reduced/';
FILEPATH_orb_vo = '~/LSV-SLAM/results_and_matlab/results/TUM_monoVO/results_orb_vo/';
FILEPATH_orb_slam = '~/LSV-SLAM/results_and_matlab/results/TUM_monoVO/results_orb_slam/';


% [2] Select mode:
% - 1: Plot pre-evaluated results to make the same figures in the paper.
% - 2: Process your own results and plot.
% - 3: Plot your processed results (if you've already run with mode=2).
mode_TUM_monoVO = 3;

% [3] If mode = 2, Select systems to evaluate:
% - 1: Our VO 
% - 2: Our SLAM
% - 3: DSO default
% - 4: DSO reduced 
% - 5: ORB-VO
% - 6: ORB-SLAM
systems_to_evaluate_TUM_monoVO = [1,2,3,4,5,6];


% [4] Run !!
switch mode_TUM_monoVO
    case 1
        disp('[TUM_monoVO] Mode 1: Plotting Pre-Evaluated Results...')
        load PRE_EVALUATED_RESULTS.mat;    
        PlotResults_TUM_monoVO;
    
    case 2
        disp('[TUM_monoVO] Mode 2: Process and Plot...')
        ProcessResults_TUM_monoVO;
        load DSO_DEFAULT_RESULTS_TUM_monoVO.mat; 
        load DSO_REDUCED_RESULTS_TUM_monoVO.mat;
        load ORB_VO_RESULTS_TUM_monoVO.mat;
        load ORB_SLAM_RESULTS_TUM_monoVO.mat;
        load OUR_VO_RESULTS_TUM_monoVO.mat;
        load OUR_SLAM_RESULTS_TUM_monoVO.mat;
        PlotResults_TUM_monoVO;
    case 3
        disp('[TUM_monoVO] Mode 3: Plotting Your Pre-Processed Results...')
        load DSO_DEFAULT_RESULTS_TUM_monoVO.mat; 
        load DSO_REDUCED_RESULTS_TUM_monoVO.mat;
        load ORB_VO_RESULTS_TUM_monoVO.mat;
        load ORB_SLAM_RESULTS_TUM_monoVO.mat;
        load OUR_VO_RESULTS_TUM_monoVO.mat;
        load OUR_SLAM_RESULTS_TUM_monoVO.mat;
        PlotResults_TUM_monoVO;
end


%% Save as PRE_EVALUATED_RESULTS
% 
% clear all; clc;
% load DSO_DEFAULT_RESULTS_TUM_monoVO.mat; 
% load DSO_REDUCED_RESULTS_TUM_monoVO.mat;
% load ORB_VO_RESULTS_TUM_monoVO.mat;
% load ORB_SLAM_RESULTS_TUM_monoVO.mat;
% load OUR_VO_RESULTS_TUM_monoVO.mat;
% load OUR_SLAM_RESULTS_TUM_monoVO.mat;
% 
% load DSO_DEFAULT_RESULTS_EuRoC_MAV.mat; 
% load DSO_REDUCED_RESULTS_EuRoC_MAV.mat;
% load ORB_VO_RESULTS_EuRoC_MAV.mat;
% load ORB_SLAM_RESULTS_EuRoC_MAV.mat;
% load OUR_VO_RESULTS_EuRoC_MAV.mat;
% load OUR_SLAM_RESULTS_EuRoC_MAV.mat;
% 
% save PRE_EVALUATED_RESULTS.mat
