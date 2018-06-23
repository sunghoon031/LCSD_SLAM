function [ my_data, gt_data, first_line, final_ts, total_map_points, nSkippedFrames, nTotalFramesAfterInit, trackingTimeMed, trackingTimeAvg, trackingTimeStd, nLoopClosures, trackingStartTimestamp, trackingEndTimestamp, startTimestamp, endTimestamp ] = LoadFromFile( my_filename, my_filename_TR, gt_filename )
%% Load groundtruth:

% Read the first line and save individual cells delimited by comma
first_line = regexp(fgetl(fopen(gt_filename)), ',', 'split');
% Load the rest of the data
% gt_data = csvread(gt_filename,1,0);
gt_data = importdata(gt_filename, ',');
gt_data = gt_data.data;
% Make timestamp in second
gt_data(:,1) = gt_data(:,1) * 1e-9;

final_ts = gt_data(end,1);

total_map_points = 0;
nSkippedFrames = 0;
nTotalFramesAfterInit = 0;
trackingTimeMed = 0;
trackingTimeAvg = 0;
trackingTimeStd = 0;
nLoopClosures = 0;
trackingStartTimestamp =  0;
trackingEndTimestamp = 0;
startTimestamp = 0;
endTimestamp = 1;

fclose all;

%% Load my result:
if ~exist(my_filename, 'file')
    my_data = [];

    return;
end

delimiter = ' '; %or whatever
fid = fopen(my_filename,'rt');
tLines = fgets(fid);
numCols = numel(strfind(tLines,delimiter)) + 1;
fclose(fid);

numColsTR = 0;
if (~isempty(my_filename_TR) && exist(my_filename_TR, 'file'))
    delimiter = ' '; %or whatever
    fid = fopen(my_filename_TR,'rt');
    tLines = fgets(fid);
    numColsTR = numel(strfind(tLines,delimiter)) + 1;
    fclose(fid);
end

my_data = [];
my_data_TR = [];

if (exist(my_filename_TR, 'file') && numColsTR == 9) % My system
    my_data_TR = cell2mat(textscan(fopen(my_filename_TR), '%f %f %f %f %f %f %f %f %f'));
    nSkippedFrames =  my_data_TR(1,1);
    nTotalFramesAfterInit =  my_data_TR(1,2);
    trackingTimeMed =  my_data_TR(1,3);
    trackingTimeAvg =  my_data_TR(1,4);
    trackingTimeStd =  my_data_TR(1,5);
    trackingStartTimestamp =  my_data_TR(1,6);
    trackingEndTimestamp = my_data_TR(1,7);
    startTimestamp = my_data_TR(1,8);
    endTimestamp = my_data_TR(1,9);
    
    if numCols == 10
        my_data = cell2mat(textscan(fopen(my_filename), '%f %f %f %f %f %f %f %f %f %f'));
        total_map_points = my_data(1,9);
        nLoopClosures = my_data(1,10);
        my_data = my_data(:,1:8);        
        
    end
    
elseif numCols == 17 %DSO
    my_data = cell2mat(textscan(fopen(my_filename), '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f'));
    nSkippedFrames =  my_data(1,9);
    nTotalFramesAfterInit =  my_data(1,10);
    trackingTimeMed =  my_data(1,11);
    trackingTimeAvg =  my_data(1,12);
    trackingTimeStd =  my_data(1,13);
    trackingStartTimestamp =  my_data(1,14);
    trackingEndTimestamp = my_data(1,15);
    startTimestamp = my_data(1,16);
    endTimestamp = my_data(1,17);
    my_data = my_data(:,1:8);

elseif numCols == 19
    my_data = cell2mat(textscan(fopen(my_filename), '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f'));
    total_map_points = my_data(1,9);
    nSkippedFrames =  my_data(1,10);
    nTotalFramesAfterInit =  my_data(1,11);
    trackingTimeMed =  my_data(1,12);
    trackingTimeAvg =  my_data(1,13);
    trackingTimeStd =  my_data(1,14);
    nLoopClosures = my_data(1,15);
    trackingStartTimestamp =  my_data(1,16);
    trackingEndTimestamp = my_data(1,17);
    startTimestamp = my_data(1,18);
    endTimestamp = my_data(1,19);
    my_data = my_data(:,1:8);
    
end


fclose all;

if isempty(my_data)
    return;
end


end

