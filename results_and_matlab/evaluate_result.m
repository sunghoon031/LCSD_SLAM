function [my_data, gt_data_sync, first_line, final_ts, ATE, nKFs, last_tracking_timestamp, total_map_points,  nSkippedFrames, nTotalFramesAfterInit, trackingTimeMed, trackingTimeAvg, trackingTimeStd, nLoopClosures,  trackingStartTimestamp, trackingEndTimestamp, startTimestamp, endTimestamp] = evaluate_result( my_filename, my_filename_TR, gt_filename )

[  my_data, gt_data, first_line, final_ts, total_map_points, nSkippedFrames, nTotalFramesAfterInit, trackingTimeMed, trackingTimeAvg, trackingTimeStd, nLoopClosures,  trackingStartTimestamp, trackingEndTimestamp, startTimestamp, endTimestamp] = LoadFromFile(my_filename, my_filename_TR, gt_filename);

gt_data_sync = [];

nKFs = size(my_data,1);
if (nKFs == 0)
    ATE = 10;
    last_tracking_timestamp = 0;
    return;
end


last_tracking_timestamp = my_data(end, 1);
% Reorder the quaternion in my_data from x,y,z,w to w,x,y,z:
x_temp = my_data(:,5);
y_temp = my_data(:,6);
z_temp = my_data(:,7);
w_temp = my_data(:,8);

my_data(:,5) = w_temp;
my_data(:,6) = x_temp;
my_data(:,7) = y_temp;
my_data(:,8) = z_temp;


%% Get corresponding Ground truth
gt_data_sync = GetCorrespondingGroundtruth(my_data, gt_data);

ATE = ComputeATE(my_data, gt_data_sync);


end

