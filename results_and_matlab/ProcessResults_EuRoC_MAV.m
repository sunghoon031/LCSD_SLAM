initialize_variables;

for system = systems_to_evaluate_EuRoC_MAV
 
    if system == 1
        pre_filename_full = [our_vo_dir, 'KeyFrameTrajectory_our_VO'];
        pre_filename_TR = [our_vo_dir, 'TrackingStats_our_VO'];
    elseif system == 2
        pre_filename_full = [our_slam_dir, 'KeyFrameTrajectory_our_SLAM'];
        pre_filename_TR = [our_slam_dir, 'TrackingStats_our_SLAM'];
    elseif system == 3
        pre_filename_full = [dso_default_dir, 'KeyFrameTrajectory_DSO_full'];
        pre_filename_half = [dso_default_dir, 'KeyFrameTrajectory_DSO_half'];
        pre_filename_TR = '';
    elseif system == 4
        pre_filename_full = [dso_reduced_dir, 'KeyFrameTrajectory_DSO_full'];
        pre_filename_half = [dso_reduced_dir, 'KeyFrameTrajectory_DSO_half'];
        pre_filename_TR = '';
    elseif system == 5
        pre_filename_full = [orb_vo_dir, 'KeyFrameTrajectory_ORB_full'];
        pre_filename_half = [orb_vo_dir, 'KeyFrameTrajectory_ORB_half'];
        pre_filename_TR = '';
    elseif system == 6
        pre_filename_full = [orb_slam_dir,'KeyFrameTrajectory_ORB_full'];
        pre_filename_half = [orb_slam_dir, 'KeyFrameTrajectory_ORB_half'];
        pre_filename_TR = '';
    end

    ATE_total=nan(10,22);
    nKeyframes_total=nan(10,22);
    nMapPoints_total=nan(10,22);
    nSkippedFrames_total=nan(10,22);
    nFramesAfterInit_total=nan(10,22);
    trackingTimeMed_total=nan(10,22);
    trackingTimeAvg_total=nan(10,22);
    trackingTimeStd_total=nan(10,22);
    nLoopClosures_total=nan(10,22);

    
    counter = 0;
    for i = 1:44 
        if ((system == 1 || system ==2) && i >= 23)
            break;
        end

        if (i < 23)
            gt_filename = [gt_dir, cell2mat(gt_files(ceil(i/2)))];
            filename = [pre_filename_full, cell2mat(euroc_sequences(i))];
            if (~isempty(pre_filename_TR))
                filename_TR = [pre_filename_TR, cell2mat(euroc_sequences(i))];
            else
                filename_TR = '';
            end
        else
            filename = [pre_filename_half, cell2mat(euroc_sequences(i-22))];
            filename_TR = '';
            gt_filename = [gt_dir, cell2mat(gt_files(ceil((i-22)/2)))];
        end
        
        counter = counter + 1;
        
        ATE_sequence = nan(10,1);
        nKeyframes_sequence=nan(10,1);
        nMapPoints_sequence=nan(10,1);
        nSkippedFrames_sequence=nan(10,1);
        nFramesAfterInit_sequence=nan(10,1);
        trackingTimeMed_sequence=nan(10,1);
        trackingTimeAvg_sequence=nan(10,1);
        trackingTimeStd_sequence=nan(10,1);
        nLoopClosures_sequence=nan(10,1);
            
        for j = 1:10
            my_filename = [filename, num2str(j), '.txt'];
            my_filename_TR = [filename_TR, num2str(j), '.txt'];
            [my_data, gt_data_sync, ...
                first_line, ...
                final_ts, ...
                ATE, ...
                nKFs, ...
                last_KF_ts, ...
                total_map_points,  ...
                nSkippedFrames, ...
                nTotalFramesAfterInit, ...
                trackingTimeMed, ...
                trackingTimeAvg, ...
                trackingTimeStd, ...
                nLoopClosures, ... 
                trackingStartTimestamp, ...
                trackingEndTimestamp, ...
                startTimestamp, ...
                endTimestamp] = evaluate_result( my_filename, my_filename_TR, gt_filename );  

            if (isnan(ATE) || (trackingEndTimestamp-trackingStartTimestamp)/(endTimestamp-startTimestamp) < 0.8 )
                ATE = 10;
            end
            
            ATE_sequence(j) = ATE;
            nKeyframes_sequence(j)= nKFs;
            nMapPoints_sequence(j)=total_map_points;
            nSkippedFrames_sequence(j)=nSkippedFrames;
            nFramesAfterInit_sequence(j)=nTotalFramesAfterInit;
            trackingTimeMed_sequence(j)=trackingTimeMed;
            trackingTimeAvg_sequence(j)=trackingTimeAvg;
            trackingTimeStd_sequence(j)=trackingTimeStd;
            nLoopClosures_sequence(j)=nLoopClosures;
        end
        
        ATE_total(:,i) = ATE_sequence;
        nKeyframes_total(:,i)= nKeyframes_sequence;
        nMapPoints_total(:,i)= nMapPoints_sequence;
        nSkippedFrames_total(:,i)=nSkippedFrames_sequence;
        nFramesAfterInit_total(:,i)= nFramesAfterInit_sequence;
        trackingTimeMed_total(:,i) = trackingTimeMed_sequence;
        trackingTimeAvg_total(:,i)= trackingTimeAvg_sequence;
        trackingTimeStd_total(:,i)= trackingTimeStd_sequence;
        nLoopClosures_total(:,i)=nLoopClosures_sequence;
            
    
        if (system == 1)
            progress = counter/22*100;
            disp(['[Our VO] Progress = ', num2str(progress), '%...'])
        elseif (system == 2)
            progress = counter/22*100;
            disp(['[Our SLAM] Progress = ', num2str(progress), '%...'])
        elseif (system == 3)
            progress = counter/44*100;
            disp(['[DSO default] Progress = ', num2str(progress), '%...'])
        elseif (system == 4)
            progress = counter/44*100;
            disp(['[DSO reduced] Progress = ', num2str(progress), '%...'])
        elseif (system == 5)
            progress = counter/44*100;
            disp(['[ORB-VO] Progress = ', num2str(progress), '%...'])
        elseif (system == 6)
            progress = counter/44*100;
            disp(['[ORB-SLAM] Progress = ', num2str(progress), '%...'])
        end
        
    end

    save_results;
   
   
end
