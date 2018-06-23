% Take averages for each sequence (combining left and right cam)

%% Ours

% Get valid average for each sequence (left and right combined)
nKeyframes_our_VO(ATEs_our_VO==10) = nan; nKeyframes_our_VO_temp = nanmean(nKeyframes_our_VO); 
nKeyframes_our_VO = nanmean(reshape([nKeyframes_our_VO_temp(:); nan(mod(-numel(nKeyframes_our_VO_temp),2),1)],2,[]));
clear nKeyframes_our_VO_temp;
nKeyframes_our_SLAM(ATEs_our_SLAM==10) = nan; nKeyframes_our_SLAM_temp = nanmean(nKeyframes_our_SLAM); 
nKeyframes_our_SLAM = nanmean(reshape([nKeyframes_our_SLAM_temp(:); nan(mod(-numel(nKeyframes_our_SLAM_temp),2),1)],2,[]));
clear nKeyframes_our_SLAM_temp;
nMapPoints_our_VO(ATEs_our_VO==10) = nan; nMapPoints_our_VO_temp = nanmean(nMapPoints_our_VO); 
nMapPoints_our_VO = nanmean(reshape([nMapPoints_our_VO_temp(:); nan(mod(-numel(nMapPoints_our_VO_temp),2),1)],2,[]));
clear nMapPoints_our_VO_temp;
nMapPoints_our_SLAM(ATEs_our_SLAM==10) = nan; nMapPoints_our_SLAM_temp = nanmean(nMapPoints_our_SLAM); 
nMapPoints_our_SLAM = nanmean(reshape([nMapPoints_our_SLAM_temp(:); nan(mod(-numel(nMapPoints_our_SLAM_temp),2),1)],2,[]));
clear nMapPoints_our_SLAM_temp;
nSkippedFrames_our_VO(ATEs_our_VO==10) = nan; nSkippedFrames_our_VO_temp = nanmean(nSkippedFrames_our_VO); 
nSkippedFrames_our_VO = nanmean(reshape([nSkippedFrames_our_VO_temp(:); nan(mod(-numel(nSkippedFrames_our_VO_temp),2),1)],2,[]));
clear nSkippedFrames_our_VO_temp;
nSkippedFrames_our_SLAM(ATEs_our_SLAM==10) = nan; nSkippedFrames_our_SLAM_temp = nanmean(nSkippedFrames_our_SLAM); 
nSkippedFrames_our_SLAM = nanmean(reshape([nSkippedFrames_our_SLAM_temp(:); nan(mod(-numel(nSkippedFrames_our_SLAM_temp),2),1)],2,[]));
clear nSkippedFrames_our_SLAM_temp;

nFramesAfterInit_our_VO_original = nFramesAfterInit_our_VO;
nFramesAfterInit_our_SLAM_original = nFramesAfterInit_our_SLAM;

nFramesAfterInit_our_VO(ATEs_our_VO==10) = nan; nFramesAfterInit_our_VO_temp = nanmean(nFramesAfterInit_our_VO); 
nFramesAfterInit_our_VO = nanmean(reshape([nFramesAfterInit_our_VO_temp(:); nan(mod(-numel(nFramesAfterInit_our_VO_temp),2),1)],2,[]));
clear nFramesAfterInit_our_VO_temp;
nFramesAfterInit_our_SLAM(ATEs_our_SLAM==10) = nan; nFramesAfterInit_our_SLAM_temp = nanmean(nFramesAfterInit_our_SLAM); 
nFramesAfterInit_our_SLAM = nanmean(reshape([nFramesAfterInit_our_SLAM_temp(:); nan(mod(-numel(nFramesAfterInit_our_SLAM_temp),2),1)],2,[]));
clear nFramesAfterInit_our_SLAM_temp;
trackingTimeMed_our_VO(ATEs_our_VO==10) = nan; trackingTimeMed_our_VO_temp = nanmean(trackingTimeMed_our_VO); 
trackingTimeMed_our_VO = nanmean(reshape([trackingTimeMed_our_VO_temp(:); nan(mod(-numel(trackingTimeMed_our_VO_temp),2),1)],2,[]));
clear trackingTimeMed_our_VO_temp;
trackingTimeMed_our_SLAM(ATEs_our_SLAM==10) = nan; trackingTimeMed_our_SLAM_temp = nanmean(trackingTimeMed_our_SLAM); 
trackingTimeMed_our_SLAM = nanmean(reshape([trackingTimeMed_our_SLAM_temp(:); nan(mod(-numel(trackingTimeMed_our_SLAM_temp),2),1)],2,[]));
clear trackingTimeMed_our_SLAM_temp;

trackingTimeAvg_our_VO_new = zeros(1,size(trackingTimeAvg_our_VO,2)/2);
trackingTimeAvg_our_SLAM_new = zeros(1,size(trackingTimeAvg_our_VO,2)/2);

trackingTimeAvgSq_our_VO_new = zeros(1,size(trackingTimeAvg_our_VO,2)/2);
trackingTimeAvgSq_our_SLAM_new = zeros(1,size(trackingTimeAvg_our_VO,2)/2);

trackingTimeVar_our_VO_new = zeros(1,size(trackingTimeStd_our_VO,2)/2);
trackingTimeVar_our_SLAM_new = zeros(1,size(trackingTimeStd_our_VO,2)/2);

trackingTimeStd_our_VO_new = zeros(1,size(trackingTimeStd_our_VO,2)/2);
trackingTimeStd_our_SLAM_new = zeros(1,size(trackingTimeStd_our_VO,2)/2);

nFramesAfterInit_our_VO_sum =0;
nFramesAfterInit_our_SLAM_sum =0;

trackingTimeAvg_our_VO_euroc = 0;
trackingTimeAvg_our_SLAM_euroc = 0;
nFramesAfterInit_our_VO_euroc=0;
nFramesAfterInit_our_SLAM_euroc=0;
trackingTimeAvgSq_our_VO_euroc = 0;
trackingTimeAvgSq_our_SLAM_euroc = 0;
trackingTimeVar_our_VO_euroc = 0;
trackingTimeVar_our_SLAM_euroc = 0;

for col =1:size(trackingTimeAvg_our_VO,2)
    
    if (rem(col,2)==1)
        nFramesAfterInit_our_VO_sum =0;
        nFramesAfterInit_our_SLAM_sum =0;
        col_new = fix(col/2)+1;
    else
        col_new = fix(col/2);
    end
    for row = 1:10
        if (ATEs_our_VO(row,col) < 10)
            trackingTimeAvg_our_VO_new(1,col_new) = trackingTimeAvg_our_VO_new(1,col_new) + ...
                trackingTimeAvg_our_VO(row,col)*nFramesAfterInit_our_VO_original(row,col);
            
            trackingTimeAvgSq_our_VO_new(1,col_new) = trackingTimeAvgSq_our_VO_new(1,col_new) + ...
                (trackingTimeAvg_our_VO(row,col))^2*nFramesAfterInit_our_VO_original(row,col);
            
            nFramesAfterInit_our_VO_sum = nFramesAfterInit_our_VO_sum+nFramesAfterInit_our_VO_original(row,col);
            
            trackingTimeVar_our_VO_new(1,col_new) = trackingTimeVar_our_VO_new(1,col_new) + ...
                (trackingTimeStd_our_VO(row,col))^2*nFramesAfterInit_our_VO_original(row,col);
            
        end
        if (ATEs_our_SLAM(row,col) < 10)
            trackingTimeAvg_our_SLAM_new(1,col_new) = trackingTimeAvg_our_SLAM_new(1,col_new) + ...
                trackingTimeAvg_our_SLAM(row,col)*nFramesAfterInit_our_SLAM_original(row,col);
            
            trackingTimeAvgSq_our_SLAM_new(1,col_new) = trackingTimeAvgSq_our_SLAM_new(1,col_new) + ...
                (trackingTimeAvg_our_SLAM(row,col))^2*nFramesAfterInit_our_SLAM_original(row,col);
            
            nFramesAfterInit_our_SLAM_sum = nFramesAfterInit_our_SLAM_sum+nFramesAfterInit_our_SLAM_original(row,col);
            
            trackingTimeVar_our_SLAM_new(1,col_new) = trackingTimeVar_our_SLAM_new(1,col_new) + ...
                (trackingTimeStd_our_SLAM(row,col))^2*nFramesAfterInit_our_SLAM_original(row,col);
        end

        
        if (row == 10 && rem(col,2)==0)
            trackingTimeAvg_our_VO_euroc = trackingTimeAvg_our_VO_euroc + trackingTimeAvg_our_VO_new(1,col_new);
            trackingTimeAvg_our_SLAM_euroc = trackingTimeAvg_our_SLAM_euroc + trackingTimeAvg_our_SLAM_new(1,col_new);
            nFramesAfterInit_our_VO_euroc = nFramesAfterInit_our_VO_euroc + nFramesAfterInit_our_VO_sum;
            nFramesAfterInit_our_SLAM_euroc = nFramesAfterInit_our_SLAM_euroc + nFramesAfterInit_our_SLAM_sum;
            trackingTimeAvgSq_our_VO_euroc = trackingTimeAvgSq_our_VO_euroc + trackingTimeAvgSq_our_VO_new(1,col_new);
            trackingTimeAvgSq_our_SLAM_euroc = trackingTimeAvgSq_our_SLAM_euroc + trackingTimeAvgSq_our_SLAM_new(1,col_new);
            trackingTimeVar_our_VO_euroc = trackingTimeVar_our_VO_euroc + trackingTimeVar_our_VO_new(1,col_new);
            trackingTimeVar_our_SLAM_euroc = trackingTimeVar_our_SLAM_euroc + trackingTimeVar_our_SLAM_new(1,col_new);
            
            trackingTimeAvg_our_VO_new(1,col_new) = trackingTimeAvg_our_VO_new(1,col_new)/nFramesAfterInit_our_VO_sum;
            trackingTimeAvg_our_SLAM_new(1,col_new) = trackingTimeAvg_our_SLAM_new(1,col_new)/nFramesAfterInit_our_SLAM_sum;
            trackingTimeAvgSq_our_VO_new(1,col_new) = trackingTimeAvgSq_our_VO_new(1,col_new)/nFramesAfterInit_our_VO_sum;
            trackingTimeAvgSq_our_SLAM_new(1,col_new) = trackingTimeAvgSq_our_SLAM_new(1,col_new)/nFramesAfterInit_our_SLAM_sum;
            trackingTimeVar_our_VO_new(1,col_new) = trackingTimeVar_our_VO_new(1,col_new)/nFramesAfterInit_our_VO_sum;
            trackingTimeVar_our_SLAM_new(1,col_new) = trackingTimeVar_our_SLAM_new(1,col_new)/nFramesAfterInit_our_SLAM_sum;
            
            trackingTimeVar_our_VO_new(1,col_new) = ...
                trackingTimeVar_our_VO_new(1,col_new)+...
                trackingTimeAvgSq_our_VO_new(1,col_new)-...
                (trackingTimeAvg_our_VO_new(1,col_new))^2;
            
            trackingTimeVar_our_SLAM_new(1,col_new) = ...
                trackingTimeVar_our_SLAM_new(1,col_new)+...
                trackingTimeAvgSq_our_SLAM_new(1,col_new)-...
                (trackingTimeAvg_our_SLAM_new(1,col_new))^2;

            
            trackingTimeStd_our_VO_new(1,col_new) =  sqrt(trackingTimeVar_our_VO_new(1,col_new));
            trackingTimeStd_our_SLAM_new(1,col_new) =  sqrt(trackingTimeVar_our_SLAM_new(1,col_new));
        end
    end
end
trackingTimeAvg_our_VO_euroc = trackingTimeAvg_our_VO_euroc/nFramesAfterInit_our_VO_euroc;
trackingTimeAvg_our_SLAM_euroc = trackingTimeAvg_our_SLAM_euroc/nFramesAfterInit_our_SLAM_euroc;
trackingTimeAvgSq_our_VO_euroc = trackingTimeAvgSq_our_VO_euroc/nFramesAfterInit_our_VO_euroc;
trackingTimeAvgSq_our_SLAM_euroc = trackingTimeAvgSq_our_SLAM_euroc/nFramesAfterInit_our_SLAM_euroc;
trackingTimeVar_our_VO_euroc = trackingTimeVar_our_VO_euroc/nFramesAfterInit_our_VO_euroc;
trackingTimeVar_our_SLAM_euroc = trackingTimeVar_our_SLAM_euroc/nFramesAfterInit_our_SLAM_euroc;

trackingTimeStd_our_VO_euroc = sqrt(trackingTimeVar_our_VO_euroc + trackingTimeAvgSq_our_VO_euroc - trackingTimeAvg_our_VO_euroc^2);
trackingTimeStd_our_SLAM_euroc = sqrt(trackingTimeVar_our_SLAM_euroc + trackingTimeAvgSq_our_SLAM_euroc - trackingTimeAvg_our_SLAM_euroc^2);

trackingTimeMed_our_VO_euroc = median(trackingTimeMed_our_VO);
trackingTimeMed_our_SLAM_euroc = median(trackingTimeMed_our_SLAM);

%% ORB
nKeyframes_orb_VO_full(ATEs_orb_VO_full==10) = nan; nKeyframes_orb_VO_full_temp = nanmean(nKeyframes_orb_VO_full); 
nKeyframes_orb_VO_full = nanmean(reshape([nKeyframes_orb_VO_full_temp(:); nan(mod(-numel(nKeyframes_orb_VO_full_temp),2),1)],2,[]));
clear nKeyframes_orb_VO_full_temp;
nKeyframes_orb_VO_half(ATEs_orb_VO_half==10) = nan; nKeyframes_orb_VO_half_temp = nanmean(nKeyframes_orb_VO_half); 
nKeyframes_orb_VO_half = nanmean(reshape([nKeyframes_orb_VO_half_temp(:); nan(mod(-numel(nKeyframes_orb_VO_half_temp),2),1)],2,[]));
clear nKeyframes_orb_VO_half_temp;
nKeyframes_orb_SLAM_full(ATEs_orb_SLAM_full==10) = nan; nKeyframes_orb_SLAM_full_temp = nanmean(nKeyframes_orb_SLAM_full); 
nKeyframes_orb_SLAM_full = nanmean(reshape([nKeyframes_orb_SLAM_full_temp(:); nan(mod(-numel(nKeyframes_orb_SLAM_full_temp),2),1)],2,[]));
clear nKeyframes_orb_SLAM_full_temp;
nKeyframes_orb_SLAM_half(ATEs_orb_SLAM_half==10) = nan; nKeyframes_orb_SLAM_half_temp = nanmean(nKeyframes_orb_SLAM_half); 
nKeyframes_orb_SLAM_half = nanmean(reshape([nKeyframes_orb_SLAM_half_temp(:); nan(mod(-numel(nKeyframes_orb_SLAM_half_temp),2),1)],2,[]));
clear nKeyframes_orb_SLAM_half_temp;
nMapPoints_orb_VO_full(ATEs_orb_VO_full==10) = nan; nMapPoints_orb_VO_full_temp = nanmean(nMapPoints_orb_VO_full); 
nMapPoints_orb_VO_full = nanmean(reshape([nMapPoints_orb_VO_full_temp(:); nan(mod(-numel(nMapPoints_orb_VO_full_temp),2),1)],2,[]));
clear nMapPoints_orb_VO_full_temp;
nMapPoints_orb_VO_half(ATEs_orb_VO_half==10) = nan; nMapPoints_orb_VO_half_temp = nanmean(nMapPoints_orb_VO_half); 
nMapPoints_orb_VO_half = nanmean(reshape([nMapPoints_orb_VO_half_temp(:); nan(mod(-numel(nMapPoints_orb_VO_half_temp),2),1)],2,[]));
clear nMapPoints_orb_VO_half_temp;
nMapPoints_orb_SLAM_full(ATEs_orb_SLAM_full==10) = nan; nMapPoints_orb_SLAM_full_temp = nanmean(nMapPoints_orb_SLAM_full); 
nMapPoints_orb_SLAM_full = nanmean(reshape([nMapPoints_orb_SLAM_full_temp(:); nan(mod(-numel(nMapPoints_orb_SLAM_full_temp),2),1)],2,[]));
clear nMapPoints_orb_SLAM_full_temp;
nMapPoints_orb_SLAM_half(ATEs_orb_SLAM_half==10) = nan; nMapPoints_orb_SLAM_half_temp = nanmean(nMapPoints_orb_SLAM_half); 
nMapPoints_orb_SLAM_half = nanmean(reshape([nMapPoints_orb_SLAM_half_temp(:); nan(mod(-numel(nMapPoints_orb_SLAM_half_temp),2),1)],2,[]));
clear nMapPoints_orb_SLAM_half_temp;
nSkippedFrames_orb_VO_full(ATEs_orb_VO_full==10) = nan; nSkippedFrames_orb_VO_full_temp = nanmean(nSkippedFrames_orb_VO_full); 
nSkippedFrames_orb_VO_full = nanmean(reshape([nSkippedFrames_orb_VO_full_temp(:); nan(mod(-numel(nSkippedFrames_orb_VO_full_temp),2),1)],2,[]));
clear nSkippedFrames_orb_VO_full_temp;
nSkippedFrames_orb_VO_half(ATEs_orb_VO_half==10) = nan; nSkippedFrames_orb_VO_half_temp = nanmean(nSkippedFrames_orb_VO_half); 
nSkippedFrames_orb_VO_half = nanmean(reshape([nSkippedFrames_orb_VO_half_temp(:); nan(mod(-numel(nSkippedFrames_orb_VO_half_temp),2),1)],2,[]));
clear nSkippedFrames_orb_VO_half_temp;
nSkippedFrames_orb_SLAM_full(ATEs_orb_SLAM_full==10) = nan; nSkippedFrames_orb_SLAM_full_temp = nanmean(nSkippedFrames_orb_SLAM_full); 
nSkippedFrames_orb_SLAM_full = nanmean(reshape([nSkippedFrames_orb_SLAM_full_temp(:); nan(mod(-numel(nSkippedFrames_orb_SLAM_full_temp),2),1)],2,[]));
clear nSkippedFrames_orb_SLAM_full_temp;
nSkippedFrames_orb_SLAM_half(ATEs_orb_SLAM_half==10) = nan; nSkippedFrames_orb_SLAM_half_temp = nanmean(nSkippedFrames_orb_SLAM_half); 
nSkippedFrames_orb_SLAM_half = nanmean(reshape([nSkippedFrames_orb_SLAM_half_temp(:); nan(mod(-numel(nSkippedFrames_orb_SLAM_half_temp),2),1)],2,[]));
clear nSkippedFrames_orb_SLAM_half_temp;

nFramesAfterInit_orb_VO_full_original = nFramesAfterInit_orb_VO_full;
nFramesAfterInit_orb_VO_half_original = nFramesAfterInit_orb_VO_half;
nFramesAfterInit_orb_SLAM_full_original = nFramesAfterInit_orb_SLAM_full;
nFramesAfterInit_orb_SLAM_half_original = nFramesAfterInit_orb_SLAM_half;

nFramesAfterInit_orb_VO_full(ATEs_orb_VO_full==10) = nan; nFramesAfterInit_orb_VO_full_temp = nanmean(nFramesAfterInit_orb_VO_full); 
nFramesAfterInit_orb_VO_full = nanmean(reshape([nFramesAfterInit_orb_VO_full_temp(:); nan(mod(-numel(nFramesAfterInit_orb_VO_full_temp),2),1)],2,[]));
clear nFramesAfterInit_orb_VO_full_temp;
nFramesAfterInit_orb_VO_half(ATEs_orb_VO_half==10) = nan; nFramesAfterInit_orb_VO_half_temp = nanmean(nFramesAfterInit_orb_VO_half); 
nFramesAfterInit_orb_VO_half = nanmean(reshape([nFramesAfterInit_orb_VO_half_temp(:); nan(mod(-numel(nFramesAfterInit_orb_VO_half_temp),2),1)],2,[]));
clear nFramesAfterInit_orb_VO_half_temp;
nFramesAfterInit_orb_SLAM_full(ATEs_orb_SLAM_full==10) = nan; nFramesAfterInit_orb_SLAM_full_temp = nanmean(nFramesAfterInit_orb_SLAM_full); 
nFramesAfterInit_orb_SLAM_full = nanmean(reshape([nFramesAfterInit_orb_SLAM_full_temp(:); nan(mod(-numel(nFramesAfterInit_orb_SLAM_full_temp),2),1)],2,[]));
clear nFramesAfterInit_orb_SLAM_full_temp;
nFramesAfterInit_orb_SLAM_half(ATEs_orb_SLAM_half==10) = nan; nFramesAfterInit_orb_SLAM_half_temp = nanmean(nFramesAfterInit_orb_SLAM_half); 
nFramesAfterInit_orb_SLAM_half = nanmean(reshape([nFramesAfterInit_orb_SLAM_half_temp(:); nan(mod(-numel(nFramesAfterInit_orb_SLAM_half_temp),2),1)],2,[]));
clear nFramesAfterInit_orb_SLAM_half_temp;
trackingTimeMed_orb_VO_full(ATEs_orb_VO_full==10) = nan; trackingTimeMed_orb_VO_full_temp = nanmean(trackingTimeMed_orb_VO_full); 
trackingTimeMed_orb_VO_full = nanmean(reshape([trackingTimeMed_orb_VO_full_temp(:); nan(mod(-numel(trackingTimeMed_orb_VO_full_temp),2),1)],2,[]));
clear trackingTimeMed_orb_VO_full_temp;
trackingTimeMed_orb_VO_half(ATEs_orb_VO_half==10) = nan; trackingTimeMed_orb_VO_half_temp = nanmean(trackingTimeMed_orb_VO_half); 
trackingTimeMed_orb_VO_half = nanmean(reshape([trackingTimeMed_orb_VO_half_temp(:); nan(mod(-numel(trackingTimeMed_orb_VO_half_temp),2),1)],2,[]));
clear trackingTimeMed_orb_VO_half_temp;
trackingTimeMed_orb_SLAM_full(ATEs_orb_SLAM_full==10) = nan; trackingTimeMed_orb_SLAM_full_temp = nanmean(trackingTimeMed_orb_SLAM_full); 
trackingTimeMed_orb_SLAM_full = nanmean(reshape([trackingTimeMed_orb_SLAM_full_temp(:); nan(mod(-numel(trackingTimeMed_orb_SLAM_full_temp),2),1)],2,[]));
clear trackingTimeMed_orb_SLAM_full_temp;
trackingTimeMed_orb_SLAM_half(ATEs_orb_SLAM_half==10) = nan; trackingTimeMed_orb_SLAM_half_temp = nanmean(trackingTimeMed_orb_SLAM_half); 
trackingTimeMed_orb_SLAM_half = nanmean(reshape([trackingTimeMed_orb_SLAM_half_temp(:); nan(mod(-numel(trackingTimeMed_orb_SLAM_half_temp),2),1)],2,[]));
clear trackingTimeMed_orb_SLAM_half_temp;

trackingTimeAvg_orb_VO_full_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);
trackingTimeAvg_orb_VO_half_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);
trackingTimeAvg_orb_SLAM_full_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);
trackingTimeAvg_orb_SLAM_half_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);

trackingTimeAvgSq_orb_VO_full_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);
trackingTimeAvgSq_orb_VO_half_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);
trackingTimeAvgSq_orb_SLAM_full_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);
trackingTimeAvgSq_orb_SLAM_half_new = zeros(1,size(trackingTimeAvg_orb_VO_full,2)/2);

trackingTimeVar_orb_VO_full_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);
trackingTimeVar_orb_VO_half_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);
trackingTimeVar_orb_SLAM_full_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);
trackingTimeVar_orb_SLAM_half_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);

trackingTimeStd_orb_VO_full_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);
trackingTimeStd_orb_VO_half_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);
trackingTimeStd_orb_SLAM_full_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);
trackingTimeStd_orb_SLAM_half_new = zeros(1,size(trackingTimeStd_orb_VO_full,2)/2);

nFramesAfterInit_orb_VO_full_sum =0;
nFramesAfterInit_orb_VO_half_sum =0;
nFramesAfterInit_orb_SLAM_full_sum =0;
nFramesAfterInit_orb_SLAM_half_sum =0;

trackingTimeAvg_orb_VO_full_euroc = 0;
trackingTimeAvg_orb_SLAM_full_euroc = 0;
nFramesAfterInit_orb_VO_full_euroc=0;
nFramesAfterInit_orb_SLAM_full_euroc=0;
trackingTimeAvgSq_orb_VO_full_euroc = 0;
trackingTimeAvgSq_orb_SLAM_full_euroc = 0;
trackingTimeVar_orb_VO_full_euroc = 0;
trackingTimeVar_orb_SLAM_full_euroc = 0;

for col =1:size(trackingTimeAvg_orb_VO_full,2)
    
    if (rem(col,2)==1)
        nFramesAfterInit_orb_VO_full_sum =0;
        nFramesAfterInit_orb_VO_half_sum =0;
        nFramesAfterInit_orb_SLAM_full_sum =0;
        nFramesAfterInit_orb_SLAM_half_sum =0;
        col_new = fix(col/2)+1;
    else
        col_new = fix(col/2);
    end
    for row = 1:10
        if (ATEs_orb_VO_full(row,col) < 10)
            trackingTimeAvg_orb_VO_full_new(1,col_new) = trackingTimeAvg_orb_VO_full_new(1,col_new) + ...
                trackingTimeAvg_orb_VO_full(row,col)*nFramesAfterInit_orb_VO_full_original(row,col);
            
            trackingTimeAvgSq_orb_VO_full_new(1,col_new) = trackingTimeAvgSq_orb_VO_full_new(1,col_new) + ...
                (trackingTimeAvg_orb_VO_full(row,col))^2*nFramesAfterInit_orb_VO_full_original(row,col);
            
            nFramesAfterInit_orb_VO_full_sum = nFramesAfterInit_orb_VO_full_sum+nFramesAfterInit_orb_VO_full_original(row,col);
            
            trackingTimeVar_orb_VO_full_new(1,col_new) = trackingTimeVar_orb_VO_full_new(1,col_new) + ...
                (trackingTimeStd_orb_VO_full(row,col))^2*nFramesAfterInit_orb_VO_full_original(row,col);
        end
        if (ATEs_orb_VO_half(row,col) < 10)
            trackingTimeAvg_orb_VO_half_new(1,col_new) = trackingTimeAvg_orb_VO_half_new(1,col_new) + ...
                trackingTimeAvg_orb_VO_half(row,col)*nFramesAfterInit_orb_VO_half_original(row,col);
            
            trackingTimeAvgSq_orb_VO_half_new(1,col_new) = trackingTimeAvgSq_orb_VO_half_new(1,col_new) + ...
                (trackingTimeAvg_orb_VO_half(row,col))^2*nFramesAfterInit_orb_VO_half_original(row,col);
            
            nFramesAfterInit_orb_VO_half_sum=nFramesAfterInit_orb_VO_half_sum+nFramesAfterInit_orb_VO_half_original(row,col);
            
            trackingTimeVar_orb_VO_half_new(1,col_new) = trackingTimeVar_orb_VO_half_new(1,col_new) + ...
                (trackingTimeStd_orb_VO_half(row,col))^2*nFramesAfterInit_orb_VO_half_original(row,col);
        end
        if (ATEs_orb_SLAM_full(row,col) < 10)
            trackingTimeAvg_orb_SLAM_full_new(1,col_new) = trackingTimeAvg_orb_SLAM_full_new(1,col_new) + ...
                trackingTimeAvg_orb_SLAM_full(row,col)*nFramesAfterInit_orb_SLAM_full_original(row,col);
            
            trackingTimeAvgSq_orb_SLAM_full_new(1,col_new) = trackingTimeAvgSq_orb_SLAM_full_new(1,col_new) + ...
                (trackingTimeAvg_orb_SLAM_full(row,col))^2*nFramesAfterInit_orb_SLAM_full_original(row,col);
            
            nFramesAfterInit_orb_SLAM_full_sum = nFramesAfterInit_orb_SLAM_full_sum+nFramesAfterInit_orb_SLAM_full_original(row,col);
            
            trackingTimeVar_orb_SLAM_full_new(1,col_new) = trackingTimeVar_orb_SLAM_full_new(1,col_new) + ...
                (trackingTimeStd_orb_SLAM_full(row,col))^2*nFramesAfterInit_orb_SLAM_full_original(row,col);
        end
        if (ATEs_orb_SLAM_half(row,col) < 10)
            trackingTimeAvg_orb_SLAM_half_new(1,col_new) = trackingTimeAvg_orb_SLAM_half_new(1,col_new) + ...
                trackingTimeAvg_orb_SLAM_half(row,col)*nFramesAfterInit_orb_SLAM_half_original(row,col);
            
            trackingTimeAvgSq_orb_SLAM_half_new(1,col_new) = trackingTimeAvgSq_orb_SLAM_half_new(1,col_new) + ...
                (trackingTimeAvg_orb_SLAM_half(row,col))^2*nFramesAfterInit_orb_SLAM_half_original(row,col);
            
            nFramesAfterInit_orb_SLAM_half_sum = nFramesAfterInit_orb_SLAM_half_sum+nFramesAfterInit_orb_SLAM_half_original(row,col);
            
            trackingTimeVar_orb_SLAM_half_new(1,col_new) = trackingTimeVar_orb_SLAM_half_new(1,col_new) + ...
                (trackingTimeStd_orb_SLAM_half(row,col))^2*nFramesAfterInit_orb_SLAM_half_original(row,col);
        end
        
        if (row == 10 && rem(col,2)==0)
            trackingTimeAvg_orb_VO_full_euroc = trackingTimeAvg_orb_VO_full_euroc+trackingTimeAvg_orb_VO_full_new(1,col_new);
            trackingTimeAvg_orb_SLAM_full_euroc = trackingTimeAvg_orb_SLAM_full_euroc+trackingTimeAvg_orb_SLAM_full_new(1,col_new);
            nFramesAfterInit_orb_VO_full_euroc = nFramesAfterInit_orb_VO_full_euroc + nFramesAfterInit_orb_VO_full_sum;
            nFramesAfterInit_orb_SLAM_full_euroc = nFramesAfterInit_orb_SLAM_full_euroc + nFramesAfterInit_orb_SLAM_full_sum;
            trackingTimeAvgSq_orb_VO_full_euroc = trackingTimeAvgSq_orb_VO_full_euroc + trackingTimeAvgSq_orb_VO_full_new(1,col_new);
            trackingTimeAvgSq_orb_SLAM_full_euroc = trackingTimeAvgSq_orb_SLAM_full_euroc + trackingTimeAvgSq_orb_SLAM_full_new(1,col_new);
            trackingTimeVar_orb_VO_full_euroc = trackingTimeVar_orb_VO_full_euroc + trackingTimeVar_orb_VO_full_new(1,col_new);
            trackingTimeVar_orb_SLAM_full_euroc = trackingTimeVar_orb_SLAM_full_euroc + trackingTimeVar_orb_SLAM_full_new(1,col_new);
            
            trackingTimeAvg_orb_VO_full_new(1,col_new) = trackingTimeAvg_orb_VO_full_new(1,col_new)/nFramesAfterInit_orb_VO_full_sum;
            trackingTimeAvg_orb_VO_half_new(1,col_new) = trackingTimeAvg_orb_VO_half_new(1,col_new)/nFramesAfterInit_orb_VO_half_sum;
            trackingTimeAvg_orb_SLAM_full_new(1,col_new) = trackingTimeAvg_orb_SLAM_full_new(1,col_new)/nFramesAfterInit_orb_SLAM_full_sum;
            trackingTimeAvg_orb_SLAM_half_new(1,col_new) = trackingTimeAvg_orb_SLAM_half_new(1,col_new)/nFramesAfterInit_orb_SLAM_half_sum;
            
            trackingTimeAvgSq_orb_VO_full_new(1,col_new) = trackingTimeAvgSq_orb_VO_full_new(1,col_new)/nFramesAfterInit_orb_VO_full_sum;
            trackingTimeAvgSq_orb_VO_half_new(1,col_new) = trackingTimeAvgSq_orb_VO_half_new(1,col_new)/nFramesAfterInit_orb_VO_half_sum;
            trackingTimeAvgSq_orb_SLAM_full_new(1,col_new) = trackingTimeAvgSq_orb_SLAM_full_new(1,col_new)/nFramesAfterInit_orb_SLAM_full_sum;
            trackingTimeAvgSq_orb_SLAM_half_new(1,col_new) = trackingTimeAvgSq_orb_SLAM_half_new(1,col_new)/nFramesAfterInit_orb_SLAM_half_sum;
            
            trackingTimeVar_orb_VO_full_new(1,col_new) = trackingTimeVar_orb_VO_full_new(1,col_new)/nFramesAfterInit_orb_VO_full_sum;
            trackingTimeVar_orb_VO_half_new(1,col_new) = trackingTimeVar_orb_VO_half_new(1,col_new)/nFramesAfterInit_orb_VO_half_sum;
            trackingTimeVar_orb_SLAM_full_new(1,col_new) = trackingTimeVar_orb_SLAM_full_new(1,col_new)/nFramesAfterInit_orb_SLAM_full_sum;
            trackingTimeVar_orb_SLAM_half_new(1,col_new) = trackingTimeVar_orb_SLAM_half_new(1,col_new)/nFramesAfterInit_orb_SLAM_half_sum;
            
            trackingTimeVar_orb_VO_full_new(1,col_new) = ...
                trackingTimeVar_orb_VO_full_new(1,col_new)+...
                trackingTimeAvgSq_orb_VO_full_new(1,col_new)-...
                (trackingTimeAvg_orb_VO_full_new(1,col_new))^2;

            trackingTimeVar_orb_VO_half_new(1,col_new) = ...
                trackingTimeVar_orb_VO_half_new(1,col_new)+...
                trackingTimeAvgSq_orb_VO_half_new(1,col_new)-...
                (trackingTimeAvg_orb_VO_half_new(1,col_new))^2;
            
            trackingTimeVar_orb_SLAM_full_new(1,col_new) = ...
                trackingTimeVar_orb_SLAM_full_new(1,col_new)+...
                trackingTimeAvgSq_orb_SLAM_full_new(1,col_new)-...
                (trackingTimeAvg_orb_SLAM_full_new(1,col_new))^2;

            trackingTimeVar_orb_SLAM_half_new(1,col_new) = ...
                trackingTimeVar_orb_SLAM_half_new(1,col_new)+...
                trackingTimeAvgSq_orb_SLAM_half_new(1,col_new)-...
                (trackingTimeAvg_orb_SLAM_half_new(1,col_new))^2;
            
            trackingTimeStd_orb_VO_full_new(1,col_new) =  sqrt(trackingTimeVar_orb_VO_full_new(1,col_new));
            trackingTimeStd_orb_VO_half_new(1,col_new) =  sqrt(trackingTimeVar_orb_VO_half_new(1,col_new));
            trackingTimeStd_orb_SLAM_full_new(1,col_new) =  sqrt(trackingTimeVar_orb_SLAM_full_new(1,col_new));
            trackingTimeStd_orb_SLAM_half_new(1,col_new) =  sqrt(trackingTimeVar_orb_SLAM_half_new(1,col_new));
        end
    end
end
trackingTimeAvg_orb_VO_full_euroc = trackingTimeAvg_orb_VO_full_euroc/nFramesAfterInit_orb_VO_full_euroc;
trackingTimeAvg_orb_SLAM_full_euroc = trackingTimeAvg_orb_SLAM_full_euroc/nFramesAfterInit_orb_SLAM_full_euroc;
trackingTimeAvgSq_orb_VO_full_euroc = trackingTimeAvgSq_orb_VO_full_euroc/nFramesAfterInit_orb_VO_full_euroc;
trackingTimeAvgSq_orb_SLAM_full_euroc = trackingTimeAvgSq_orb_SLAM_full_euroc/nFramesAfterInit_orb_SLAM_full_euroc;
trackingTimeVar_orb_VO_full_euroc = trackingTimeVar_orb_VO_full_euroc/nFramesAfterInit_orb_VO_full_euroc;
trackingTimeVar_orb_SLAM_full_euroc = trackingTimeVar_orb_SLAM_full_euroc/nFramesAfterInit_orb_SLAM_full_euroc;

trackingTimeStd_orb_VO_full_euroc = sqrt(trackingTimeVar_orb_VO_full_euroc + trackingTimeAvgSq_orb_VO_full_euroc - trackingTimeAvg_orb_VO_full_euroc^2);
trackingTimeStd_orb_SLAM_full_euroc = sqrt(trackingTimeVar_orb_SLAM_full_euroc + trackingTimeAvgSq_orb_SLAM_full_euroc - trackingTimeAvg_orb_SLAM_full_euroc^2);

trackingTimeMed_orb_VO_full_euroc = median(trackingTimeMed_orb_VO_full);
trackingTimeMed_orb_SLAM_full_euroc = median(trackingTimeMed_orb_SLAM_full);
%% DSO
nKeyframes_dso_default_full(ATEs_dso_default_full==10) = nan; nKeyframes_dso_default_full_temp = nanmean(nKeyframes_dso_default_full); 
nKeyframes_dso_default_full = nanmean(reshape([nKeyframes_dso_default_full_temp(:); nan(mod(-numel(nKeyframes_dso_default_full_temp),2),1)],2,[]));
clear nKeyframes_dso_default_full_temp;
nKeyframes_dso_default_half(ATEs_dso_default_half==10) = nan; nKeyframes_dso_default_half_temp = nanmean(nKeyframes_dso_default_half); 
nKeyframes_dso_default_half = nanmean(reshape([nKeyframes_dso_default_half_temp(:); nan(mod(-numel(nKeyframes_dso_default_half_temp),2),1)],2,[]));
clear nKeyframes_dso_default_half_temp;
nKeyframes_dso_reduced_full(ATEs_dso_reduced_full==10) = nan; nKeyframes_dso_reduced_full_temp = nanmean(nKeyframes_dso_reduced_full); 
nKeyframes_dso_reduced_full = nanmean(reshape([nKeyframes_dso_reduced_full_temp(:); nan(mod(-numel(nKeyframes_dso_reduced_full_temp),2),1)],2,[]));
clear nKeyframes_dso_reduced_full_temp;
nKeyframes_dso_reduced_half(ATEs_dso_reduced_half==10) = nan; nKeyframes_dso_reduced_half_temp = nanmean(nKeyframes_dso_reduced_half); 
nKeyframes_dso_reduced_half = nanmean(reshape([nKeyframes_dso_reduced_half_temp(:); nan(mod(-numel(nKeyframes_dso_reduced_half_temp),2),1)],2,[]));
clear nKeyframes_dso_reduced_half_temp;
nMapPoints_dso_default_full(ATEs_dso_default_full==10) = nan; nMapPoints_dso_default_full_temp = nanmean(nMapPoints_dso_default_full); 
nMapPoints_dso_default_full = nanmean(reshape([nMapPoints_dso_default_full_temp(:); nan(mod(-numel(nMapPoints_dso_default_full_temp),2),1)],2,[]));
clear nMapPoints_dso_default_full_temp;
nMapPoints_dso_default_half(ATEs_dso_default_half==10) = nan; nMapPoints_dso_default_half_temp = nanmean(nMapPoints_dso_default_half); 
nMapPoints_dso_default_half = nanmean(reshape([nMapPoints_dso_default_half_temp(:); nan(mod(-numel(nMapPoints_dso_default_half_temp),2),1)],2,[]));
clear nMapPoints_dso_default_half_temp;
nMapPoints_dso_reduced_full(ATEs_dso_reduced_full==10) = nan; nMapPoints_dso_reduced_full_temp = nanmean(nMapPoints_dso_reduced_full); 
nMapPoints_dso_reduced_full = nanmean(reshape([nMapPoints_dso_reduced_full_temp(:); nan(mod(-numel(nMapPoints_dso_reduced_full_temp),2),1)],2,[]));
clear nMapPoints_dso_reduced_full_temp;
nMapPoints_dso_reduced_half(ATEs_dso_reduced_half==10) = nan; nMapPoints_dso_reduced_half_temp = nanmean(nMapPoints_dso_reduced_half); 
nMapPoints_dso_reduced_half = nanmean(reshape([nMapPoints_dso_reduced_half_temp(:); nan(mod(-numel(nMapPoints_dso_reduced_half_temp),2),1)],2,[]));
clear nMapPoints_dso_reduced_half_temp;
nSkippedFrames_dso_default_full(ATEs_dso_default_full==10) = nan; nSkippedFrames_dso_default_full_temp = nanmean(nSkippedFrames_dso_default_full); 
nSkippedFrames_dso_default_full = nanmean(reshape([nSkippedFrames_dso_default_full_temp(:); nan(mod(-numel(nSkippedFrames_dso_default_full_temp),2),1)],2,[]));
clear nSkippedFrames_dso_default_full_temp;
nSkippedFrames_dso_default_half(ATEs_dso_default_half==10) = nan; nSkippedFrames_dso_default_half_temp = nanmean(nSkippedFrames_dso_default_half); 
nSkippedFrames_dso_default_half = nanmean(reshape([nSkippedFrames_dso_default_half_temp(:); nan(mod(-numel(nSkippedFrames_dso_default_half_temp),2),1)],2,[]));
clear nSkippedFrames_dso_default_half_temp;
nSkippedFrames_dso_reduced_full(ATEs_dso_reduced_full==10) = nan; nSkippedFrames_dso_reduced_full_temp = nanmean(nSkippedFrames_dso_reduced_full); 
nSkippedFrames_dso_reduced_full = nanmean(reshape([nSkippedFrames_dso_reduced_full_temp(:); nan(mod(-numel(nSkippedFrames_dso_reduced_full_temp),2),1)],2,[]));
clear nSkippedFrames_dso_reduced_full_temp;
nSkippedFrames_dso_reduced_half(ATEs_dso_reduced_half==10) = nan; nSkippedFrames_dso_reduced_half_temp = nanmean(nSkippedFrames_dso_reduced_half); 
nSkippedFrames_dso_reduced_half = nanmean(reshape([nSkippedFrames_dso_reduced_half_temp(:); nan(mod(-numel(nSkippedFrames_dso_reduced_half_temp),2),1)],2,[]));
clear nSkippedFrames_dso_reduced_half_temp;

nFramesAfterInit_dso_default_full_original = nFramesAfterInit_dso_default_full;
nFramesAfterInit_dso_default_half_original = nFramesAfterInit_dso_default_half;
nFramesAfterInit_dso_reduced_full_original = nFramesAfterInit_dso_reduced_full;
nFramesAfterInit_dso_reduced_half_original = nFramesAfterInit_dso_reduced_half;

nFramesAfterInit_dso_default_full(ATEs_dso_default_full==10) = nan; nFramesAfterInit_dso_default_full_temp = nanmean(nFramesAfterInit_dso_default_full); 
nFramesAfterInit_dso_default_full = nanmean(reshape([nFramesAfterInit_dso_default_full_temp(:); nan(mod(-numel(nFramesAfterInit_dso_default_full_temp),2),1)],2,[]));
clear nFramesAfterInit_dso_default_full_temp;
nFramesAfterInit_dso_default_half(ATEs_dso_default_half==10) = nan; nFramesAfterInit_dso_default_half_temp = nanmean(nFramesAfterInit_dso_default_half); 
nFramesAfterInit_dso_default_half = nanmean(reshape([nFramesAfterInit_dso_default_half_temp(:); nan(mod(-numel(nFramesAfterInit_dso_default_half_temp),2),1)],2,[]));
clear nFramesAfterInit_dso_default_half_temp;
nFramesAfterInit_dso_reduced_full(ATEs_dso_reduced_full==10) = nan; nFramesAfterInit_dso_reduced_full_temp = nanmean(nFramesAfterInit_dso_reduced_full); 
nFramesAfterInit_dso_reduced_full = nanmean(reshape([nFramesAfterInit_dso_reduced_full_temp(:); nan(mod(-numel(nFramesAfterInit_dso_reduced_full_temp),2),1)],2,[]));
clear nFramesAfterInit_dso_reduced_full_temp;
nFramesAfterInit_dso_reduced_half(ATEs_dso_reduced_half==10) = nan; nFramesAfterInit_dso_reduced_half_temp = nanmean(nFramesAfterInit_dso_reduced_half); 
nFramesAfterInit_dso_reduced_half = nanmean(reshape([nFramesAfterInit_dso_reduced_half_temp(:); nan(mod(-numel(nFramesAfterInit_dso_reduced_half_temp),2),1)],2,[]));
clear nFramesAfterInit_dso_reduced_half_temp;
trackingTimeMed_dso_default_full(ATEs_dso_default_full==10) = nan; trackingTimeMed_dso_default_full_temp = nanmean(trackingTimeMed_dso_default_full); 
trackingTimeMed_dso_default_full = nanmean(reshape([trackingTimeMed_dso_default_full_temp(:); nan(mod(-numel(trackingTimeMed_dso_default_full_temp),2),1)],2,[]));
clear trackingTimeMed_dso_default_full_temp;
trackingTimeMed_dso_default_half(ATEs_dso_default_half==10) = nan; trackingTimeMed_dso_default_half_temp = nanmean(trackingTimeMed_dso_default_half); 
trackingTimeMed_dso_default_half = nanmean(reshape([trackingTimeMed_dso_default_half_temp(:); nan(mod(-numel(trackingTimeMed_dso_default_half_temp),2),1)],2,[]));
clear trackingTimeMed_dso_default_half_temp;
trackingTimeMed_dso_reduced_full(ATEs_dso_reduced_full==10) = nan; trackingTimeMed_dso_reduced_full_temp = nanmean(trackingTimeMed_dso_reduced_full); 
trackingTimeMed_dso_reduced_full = nanmean(reshape([trackingTimeMed_dso_reduced_full_temp(:); nan(mod(-numel(trackingTimeMed_dso_reduced_full_temp),2),1)],2,[]));
clear trackingTimeMed_dso_reduced_full_temp;
trackingTimeMed_dso_reduced_half(ATEs_dso_reduced_half==10) = nan; trackingTimeMed_dso_reduced_half_temp = nanmean(trackingTimeMed_dso_reduced_half); 
trackingTimeMed_dso_reduced_half = nanmean(reshape([trackingTimeMed_dso_reduced_half_temp(:); nan(mod(-numel(trackingTimeMed_dso_reduced_half_temp),2),1)],2,[]));
clear trackingTimeMed_dso_reduced_half_temp;

trackingTimeAvg_dso_default_full_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);
trackingTimeAvg_dso_default_half_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);
trackingTimeAvg_dso_reduced_full_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);
trackingTimeAvg_dso_reduced_half_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);

trackingTimeAvgSq_dso_default_full_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);
trackingTimeAvgSq_dso_default_half_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);
trackingTimeAvgSq_dso_reduced_full_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);
trackingTimeAvgSq_dso_reduced_half_new = zeros(1,size(trackingTimeAvg_dso_default_full,2)/2);

trackingTimeVar_dso_default_full_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);
trackingTimeVar_dso_default_half_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);
trackingTimeVar_dso_reduced_full_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);
trackingTimeVar_dso_reduced_half_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);

trackingTimeStd_dso_default_full_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);
trackingTimeStd_dso_default_half_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);
trackingTimeStd_dso_reduced_full_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);
trackingTimeStd_dso_reduced_half_new = zeros(1,size(trackingTimeStd_dso_default_full,2)/2);

nFramesAfterInit_dso_default_full_sum =0;
nFramesAfterInit_dso_default_half_sum =0;
nFramesAfterInit_dso_reduced_full_sum =0;
nFramesAfterInit_dso_reduced_half_sum =0;

trackingTimeAvg_dso_default_full_euroc = 0;
trackingTimeAvg_dso_reduced_half_euroc = 0;
nFramesAfterInit_dso_default_full_euroc=0;
nFramesAfterInit_dso_reduced_half_euroc=0;
trackingTimeAvgSq_dso_default_full_euroc = 0;
trackingTimeAvgSq_dso_reduced_half_euroc = 0;
trackingTimeVar_dso_default_full_euroc = 0;
trackingTimeVar_dso_reduced_half_euroc = 0;

for col =1:size(trackingTimeAvg_dso_default_full,2)
    
    if (rem(col,2)==1)
        nFramesAfterInit_dso_default_full_sum =0;
        nFramesAfterInit_dso_default_half_sum =0;
        nFramesAfterInit_dso_reduced_full_sum =0;
        nFramesAfterInit_dso_reduced_half_sum =0;
        col_new = fix(col/2)+1;
    else
        col_new = fix(col/2);
    end
    for row = 1:10
        if (ATEs_dso_default_full(row,col) < 10)
            trackingTimeAvg_dso_default_full_new(1,col_new) = trackingTimeAvg_dso_default_full_new(1,col_new) + ...
                trackingTimeAvg_dso_default_full(row,col)*nFramesAfterInit_dso_default_full_original(row,col);
            
            trackingTimeAvgSq_dso_default_full_new(1,col_new) = trackingTimeAvgSq_dso_default_full_new(1,col_new) + ...
                (trackingTimeAvg_dso_default_full(row,col))^2*nFramesAfterInit_dso_default_full_original(row,col);
            
            nFramesAfterInit_dso_default_full_sum = nFramesAfterInit_dso_default_full_sum+nFramesAfterInit_dso_default_full_original(row,col);
            
            trackingTimeVar_dso_default_full_new(1,col_new) = trackingTimeVar_dso_default_full_new(1,col_new) + ...
                (trackingTimeStd_dso_default_full(row,col))^2*nFramesAfterInit_dso_default_full_original(row,col);
        end
        if (ATEs_dso_default_half(row,col) < 10)
            trackingTimeAvg_dso_default_half_new(1,col_new) = trackingTimeAvg_dso_default_half_new(1,col_new) + ...
                trackingTimeAvg_dso_default_half(row,col)*nFramesAfterInit_dso_default_half_original(row,col);
            
            trackingTimeAvgSq_dso_default_half_new(1,col_new) = trackingTimeAvgSq_dso_default_half_new(1,col_new) + ...
                (trackingTimeAvg_dso_default_half(row,col))^2*nFramesAfterInit_dso_default_half_original(row,col);
            
            nFramesAfterInit_dso_default_half_sum=nFramesAfterInit_dso_default_half_sum+nFramesAfterInit_dso_default_half_original(row,col);
            
            trackingTimeVar_dso_default_half_new(1,col_new) = trackingTimeVar_dso_default_half_new(1,col_new) + ...
                (trackingTimeStd_dso_default_half(row,col))^2*nFramesAfterInit_dso_default_half_original(row,col);
        end
        if (ATEs_dso_reduced_full(row,col) < 10)
            trackingTimeAvg_dso_reduced_full_new(1,col_new) = trackingTimeAvg_dso_reduced_full_new(1,col_new) + ...
                trackingTimeAvg_dso_reduced_full(row,col)*nFramesAfterInit_dso_reduced_full_original(row,col);
            
            trackingTimeAvgSq_dso_reduced_full_new(1,col_new) = trackingTimeAvgSq_dso_reduced_full_new(1,col_new) + ...
                (trackingTimeAvg_dso_reduced_full(row,col))^2*nFramesAfterInit_dso_reduced_full_original(row,col);
            
            nFramesAfterInit_dso_reduced_full_sum = nFramesAfterInit_dso_reduced_full_sum+nFramesAfterInit_dso_reduced_full_original(row,col);
            
            trackingTimeVar_dso_reduced_full_new(1,col_new) = trackingTimeVar_dso_reduced_full_new(1,col_new) + ...
                (trackingTimeStd_dso_reduced_full(row,col))^2*nFramesAfterInit_dso_reduced_full_original(row,col);
        end
        if (ATEs_dso_reduced_half(row,col) < 10)
            trackingTimeAvg_dso_reduced_half_new(1,col_new) = trackingTimeAvg_dso_reduced_half_new(1,col_new) + ...
                trackingTimeAvg_dso_reduced_half(row,col)*nFramesAfterInit_dso_reduced_half_original(row,col);
            
            trackingTimeAvgSq_dso_reduced_half_new(1,col_new) = trackingTimeAvgSq_dso_reduced_half_new(1,col_new) + ...
                (trackingTimeAvg_dso_reduced_half(row,col))^2*nFramesAfterInit_dso_reduced_half_original(row,col);
            
            nFramesAfterInit_dso_reduced_half_sum = nFramesAfterInit_dso_reduced_half_sum+nFramesAfterInit_dso_reduced_half_original(row,col);
            
            trackingTimeVar_dso_reduced_half_new(1,col_new) = trackingTimeVar_dso_reduced_half_new(1,col_new) + ...
                (trackingTimeStd_dso_reduced_half(row,col))^2*nFramesAfterInit_dso_reduced_half_original(row,col);
        end
        
        if (row == 10 && rem(col,2)==0)
            trackingTimeAvg_dso_default_full_euroc = trackingTimeAvg_dso_default_full_euroc+trackingTimeAvg_dso_default_full_new(1,col_new);
            trackingTimeAvg_dso_reduced_half_euroc = trackingTimeAvg_dso_reduced_half_euroc+trackingTimeAvg_dso_reduced_half_new(1,col_new);
            nFramesAfterInit_dso_default_full_euroc = nFramesAfterInit_dso_default_full_euroc + nFramesAfterInit_dso_default_full_sum;
            nFramesAfterInit_dso_reduced_half_euroc = nFramesAfterInit_dso_reduced_half_euroc + nFramesAfterInit_dso_reduced_half_sum;
            trackingTimeAvgSq_dso_default_full_euroc = trackingTimeAvgSq_dso_default_full_euroc + trackingTimeAvgSq_dso_default_full_new(1,col_new);
            trackingTimeAvgSq_dso_reduced_half_euroc = trackingTimeAvgSq_dso_reduced_half_euroc + trackingTimeAvgSq_dso_reduced_half_new(1,col_new);
            trackingTimeVar_dso_default_full_euroc = trackingTimeVar_dso_default_full_euroc + trackingTimeVar_dso_default_full_new(1,col_new);
            trackingTimeVar_dso_reduced_half_euroc = trackingTimeVar_dso_reduced_half_euroc + trackingTimeVar_dso_reduced_half_new(1,col_new);
            
            
            trackingTimeAvg_dso_default_full_new(1,col_new) = trackingTimeAvg_dso_default_full_new(1,col_new)/nFramesAfterInit_dso_default_full_sum;
            trackingTimeAvg_dso_default_half_new(1,col_new) = trackingTimeAvg_dso_default_half_new(1,col_new)/nFramesAfterInit_dso_default_half_sum;
            trackingTimeAvg_dso_reduced_full_new(1,col_new) = trackingTimeAvg_dso_reduced_full_new(1,col_new)/nFramesAfterInit_dso_reduced_full_sum;
            trackingTimeAvg_dso_reduced_half_new(1,col_new) = trackingTimeAvg_dso_reduced_half_new(1,col_new)/nFramesAfterInit_dso_reduced_half_sum;
            
            trackingTimeAvgSq_dso_default_full_new(1,col_new) = trackingTimeAvgSq_dso_default_full_new(1,col_new)/nFramesAfterInit_dso_default_full_sum;
            trackingTimeAvgSq_dso_default_half_new(1,col_new) = trackingTimeAvgSq_dso_default_half_new(1,col_new)/nFramesAfterInit_dso_default_half_sum;
            trackingTimeAvgSq_dso_reduced_full_new(1,col_new) = trackingTimeAvgSq_dso_reduced_full_new(1,col_new)/nFramesAfterInit_dso_reduced_full_sum;
            trackingTimeAvgSq_dso_reduced_half_new(1,col_new) = trackingTimeAvgSq_dso_reduced_half_new(1,col_new)/nFramesAfterInit_dso_reduced_half_sum;
            
            trackingTimeVar_dso_default_full_new(1,col_new) = trackingTimeVar_dso_default_full_new(1,col_new)/nFramesAfterInit_dso_default_full_sum;
            trackingTimeVar_dso_default_half_new(1,col_new) = trackingTimeVar_dso_default_half_new(1,col_new)/nFramesAfterInit_dso_default_half_sum;
            trackingTimeVar_dso_reduced_full_new(1,col_new) = trackingTimeVar_dso_reduced_full_new(1,col_new)/nFramesAfterInit_dso_reduced_full_sum;
            trackingTimeVar_dso_reduced_half_new(1,col_new) = trackingTimeVar_dso_reduced_half_new(1,col_new)/nFramesAfterInit_dso_reduced_half_sum;
            
            trackingTimeVar_dso_default_full_new(1,col_new) = ...
                trackingTimeVar_dso_default_full_new(1,col_new)+...
                trackingTimeAvgSq_dso_default_full_new(1,col_new)-...
                (trackingTimeAvg_dso_default_full_new(1,col_new))^2;

            trackingTimeVar_dso_default_half_new(1,col_new) = ...
                trackingTimeVar_dso_default_half_new(1,col_new)+...
                trackingTimeAvgSq_dso_default_half_new(1,col_new)-...
                (trackingTimeAvg_dso_default_half_new(1,col_new))^2;
            
            trackingTimeVar_dso_reduced_full_new(1,col_new) = ...
                trackingTimeVar_dso_reduced_full_new(1,col_new)+...
                trackingTimeAvgSq_dso_reduced_full_new(1,col_new)-...
                (trackingTimeAvg_dso_reduced_full_new(1,col_new))^2;

            trackingTimeVar_dso_reduced_half_new(1,col_new) = ...
                trackingTimeVar_dso_reduced_half_new(1,col_new)+...
                trackingTimeAvgSq_dso_reduced_half_new(1,col_new)-...
                (trackingTimeAvg_dso_reduced_half_new(1,col_new))^2;
            
            trackingTimeStd_dso_default_full_new(1,col_new) =  sqrt(trackingTimeVar_dso_default_full_new(1,col_new));
            trackingTimeStd_dso_default_half_new(1,col_new) =  sqrt(trackingTimeVar_dso_default_half_new(1,col_new));
            trackingTimeStd_dso_reduced_full_new(1,col_new) =  sqrt(trackingTimeVar_dso_reduced_full_new(1,col_new));
            trackingTimeStd_dso_reduced_half_new(1,col_new) =  sqrt(trackingTimeVar_dso_reduced_half_new(1,col_new));
        end
    end
end
trackingTimeAvg_dso_default_full_euroc = trackingTimeAvg_dso_default_full_euroc/ nFramesAfterInit_dso_default_full_euroc;
trackingTimeAvg_dso_reduced_half_euroc = trackingTimeAvg_dso_reduced_half_euroc/ nFramesAfterInit_dso_reduced_half_euroc;
trackingTimeAvgSq_dso_default_full_euroc = trackingTimeAvgSq_dso_default_full_euroc/nFramesAfterInit_dso_default_full_euroc;
trackingTimeAvgSq_dso_reduced_half_euroc = trackingTimeAvgSq_dso_reduced_half_euroc/nFramesAfterInit_dso_reduced_half_euroc;
trackingTimeVar_dso_default_full_euroc = trackingTimeVar_dso_default_full_euroc/nFramesAfterInit_dso_default_full_euroc;
trackingTimeVar_dso_reduced_half_euroc = trackingTimeVar_dso_reduced_half_euroc/nFramesAfterInit_dso_reduced_half_euroc;

trackingTimeStd_dso_default_full_euroc = sqrt(trackingTimeVar_dso_default_full_euroc + trackingTimeAvgSq_dso_default_full_euroc - trackingTimeAvg_dso_default_full_euroc^2);
trackingTimeStd_dso_reduced_half_euroc = sqrt(trackingTimeVar_dso_reduced_half_euroc + trackingTimeAvgSq_dso_reduced_half_euroc - trackingTimeAvg_dso_reduced_half_euroc^2);

trackingTimeMed_dso_default_full_euroc = median(trackingTimeMed_dso_default_full);
trackingTimeMed_dso_reduced_half_euroc = median(trackingTimeMed_dso_reduced_half);


