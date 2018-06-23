plot_factor_original = plot_factor;
linewidth_original = linewidth;
markerlinewidth_original = markerlinewidth;
boxlinewidth_original = boxlinewidth;
fontsize_original = fontsize;


plot_factor = 1;
linewidth = 4*plot_factor;
markerlinewidth = 3*plot_factor;
boxlinewidth = 3*plot_factor;
fontsize = 40*plot_factor;

%%
nSkippedFrames_ratios = ...
    [100*nSkippedFrames_orb_VO_full./nFramesAfterInit_orb_VO_full;...
    100*nSkippedFrames_orb_SLAM_full./nFramesAfterInit_orb_SLAM_full;...
    100*nSkippedFrames_dso_default_full./nFramesAfterInit_dso_default_full;...
    100*nSkippedFrames_dso_reduced_half./nFramesAfterInit_dso_reduced_half;...
    100*nSkippedFrames_our_VO./nFramesAfterInit_our_VO;...
    100*nSkippedFrames_our_SLAM./nFramesAfterInit_our_SLAM];

figure;
subplot(2,7,[1 2 3 4 5])
h=bar(nSkippedFrames_ratios);
l = cell(1,11);
l{1}='MH01'; l{2}='MH02'; l{3}='MH03'; l{4}='MH04'; l{5}='MH05';
l{6}='V101'; l{7}='V102'; l{8}='V103'; l{9}='V201'; l{10}='V202'; l{11}='V203';
legend(h,l, 'Orientation', 'horizontal', 'Location', 'northoutside');
modes = {['\begin{tabular}{c} ORB \\VO \end{tabular}'], ...
    ['\begin{tabular}{c} ORB \\SLAM \end{tabular}'], ...
    ['\begin{tabular}{c} DSO \\default \end{tabular}'], ...
    ['\begin{tabular}{c} DSO \\reduced \end{tabular}'], ...
    ['\begin{tabular}{c} Ours \\VO \end{tabular}'], ...
    ['\begin{tabular}{c} Ours \\SLAM \end{tabular}']};
set(gca,'xticklabel',modes, 'TickLength',[0 0], 'ygrid', 'on', 'TickLabelInterpreter', 'latex')
set(gca,'TickLength',[0 0])
ylabel('Missed frames (%)')
set(gca, 'Box', 'off', 'GridLineStyle',':', 'BoxStyle', 'full', 'LineWidth', boxlinewidth,'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
ylim([0 max(max(nSkippedFrames_ratios))*1.1])
xlim([0.5 6.5])

nSkippedFrames_ratio_orb_VO_full_median = median(nSkippedFrames_ratios(1,:));
nSkippedFrames_ratio_orb_SLAM_full_median = median(nSkippedFrames_ratios(2,:));
nSkippedFrames_ratio_dso_default_full_median = median(nSkippedFrames_ratios(3,:));
nSkippedFrames_ratio_dso_reduced_half_median = median(nSkippedFrames_ratios(4,:));
nSkippedFrames_ratio_our_VO_median = median(nSkippedFrames_ratios(5,:));
nSkippedFrames_ratio_our_SLAM_median = median(nSkippedFrames_ratios(6,:));
nSkippedFrames_ratio_orb_VO_full_mean = mean(nSkippedFrames_ratios(1,:));
nSkippedFrames_ratio_orb_SLAM_full_mean = mean(nSkippedFrames_ratios(2,:));
nSkippedFrames_ratio_dso_default_full_mean = mean(nSkippedFrames_ratios(3,:));
nSkippedFrames_ratio_dso_reduced_half_mean = mean(nSkippedFrames_ratios(4,:));
nSkippedFrames_ratio_our_VO_mean = mean(nSkippedFrames_ratios(5,:));
nSkippedFrames_ratio_our_SLAM_mean = mean(nSkippedFrames_ratios(6,:));
nSkippedFrames_ratio_orb_VO_full_std = std(nSkippedFrames_ratios(1,:));
nSkippedFrames_ratio_orb_SLAM_full_std = std(nSkippedFrames_ratios(2,:));
nSkippedFrames_ratio_dso_default_full_std = std(nSkippedFrames_ratios(3,:));
nSkippedFrames_ratio_dso_reduced_half_std = std(nSkippedFrames_ratios(4,:));
nSkippedFrames_ratio_our_VO_std = std(nSkippedFrames_ratios(5,:));
nSkippedFrames_ratio_our_SLAM_std = std(nSkippedFrames_ratios(6,:));


trackingTimeAvg_total = [trackingTimeAvg_orb_VO_full_new; trackingTimeAvg_orb_SLAM_full_new; ...
                        trackingTimeAvg_dso_default_full_new; trackingTimeAvg_dso_reduced_half_new;...
                        trackingTimeAvg_our_VO_new; trackingTimeAvg_our_SLAM_new];
trackingTimeStd_total = [trackingTimeStd_orb_VO_full_new; trackingTimeStd_orb_SLAM_full_new;...
                        trackingTimeStd_dso_default_full_new; trackingTimeStd_dso_reduced_half_new;...
                        trackingTimeStd_our_VO_new; trackingTimeStd_our_SLAM_new];

figure;
h=bar(trackingTimeAvg_total);
% l = cell(1,11);
% l{1}='MH01'; l{2}='MH02'; l{3}='MH03'; l{4}='MH04'; l{5}='MH05';
% l{6}='V101'; l{7}='V102'; l{8}='V103'; l{9}='V201'; l{10}='V202'; l{11}='V203';
% legend(h,l, 'Orientation', 'horizontal', 'Location', 'northoutside');
modes = {'ORB-VO','ORB-SLAM', 'DSO default', 'DSO reduced', 'Ours-VO','Ours-SLAM'};
set(gca,'xticklabel',modes, 'TickLength',[0 0], 'ygrid', 'on')
set(gca,'TickLength',[0 0])
ylabel('Mean Tracking Time (ms)')
set(gca, 'Box', 'off', 'GridLineStyle',':', 'BoxStyle', 'full', 'LineWidth', boxlinewidth,'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
ylim([0 max(max(trackingTimeAvg_total))*1.1])


hold on;
ngroups = size(trackingTimeAvg_total, 1);
nbars = size(trackingTimeAvg_total, 2);
groupwidth = min(0.8, nbars/(nbars + 1.5));
for i = 1:nbars
    % Calculate center of each bar
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    errorbar(x, trackingTimeAvg_total(:,i), trackingTimeStd_total(:,i), 'k', 'linestyle', 'none', 'LineWidth', boxlinewidth, 'CapSize', 10);
    ylim([0 max(max(trackingTimeAvg_total+trackingTimeStd_total))*1.1])

end

%%
nKeyframes_total = ...
    [nKeyframes_orb_VO_full; nKeyframes_orb_SLAM_full; nKeyframes_our_VO; nKeyframes_our_SLAM];
nKeyframes_orb_VO_mean = round(mean(nKeyframes_orb_VO_full),1);
nKeyframes_orb_SLAM_mean = round(mean(nKeyframes_orb_SLAM_full),1);
nKeyframes_our_VO_mean = round(mean(nKeyframes_our_VO),1);
nKeyframes_our_SLAM_mean = round(mean(nKeyframes_our_SLAM),1);


figure;
subplot(2,1,1)
h=bar(nKeyframes_total);
l = cell(1,11);
l{1}='MH01'; l{2}='MH02'; l{3}='MH03'; l{4}='MH04'; l{5}='MH05';
l{6}='V101'; l{7}='V102'; l{8}='V103'; l{9}='V201'; l{10}='V202'; l{11}='V203';
legend(h,l, 'Orientation', 'horizontal', 'Location', 'northoutside');
modes = {['\begin{tabular}{c} ORB-VO \\(', num2str(nKeyframes_orb_VO_mean),  ')\end{tabular}'], ...
    ['\begin{tabular}{c} ORB-SLAM \\(', num2str(nKeyframes_orb_SLAM_mean),  ')\end{tabular}'], ...
    ['\begin{tabular}{c} Ours-VO \\(', num2str(nKeyframes_our_VO_mean),  ')\end{tabular}'], ...
    ['\begin{tabular}{c} Ours-SLAM \\(', num2str(nKeyframes_our_SLAM_mean),  ')\end{tabular}']};
set(gca,'xticklabel',modes, 'TickLength',[0 0], 'ygrid', 'on', 'TickLabelInterpreter', 'latex')
set(gca,'TickLength',[0 0])
ylabel('Keyframes')
set(gca, 'Box', 'off', 'GridLineStyle',':', 'BoxStyle', 'full', 'LineWidth', boxlinewidth,'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
ylim([0 max(max(nKeyframes_total))*1.1])


nMapPoints_total = ...
    [nMapPoints_orb_VO_full; nMapPoints_orb_SLAM_full;nMapPoints_our_VO; nMapPoints_our_SLAM];
nMapPoints_orb_VO_mean = round(mean(nMapPoints_orb_VO_full),1);
nMapPoints_orb_SLAM_mean = round(mean(nMapPoints_orb_SLAM_full),1);
nMapPoints_our_VO_mean = round(mean(nMapPoints_our_VO),1);
nMapPoints_our_SLAM_mean = round(mean(nMapPoints_our_SLAM),1);

subplot(2,1,2)
h=bar(nMapPoints_total);
% l = cell(1,11);
% l{1}='MH01'; l{2}='MH02'; l{3}='MH03'; l{4}='MH04'; l{5}='MH05';
% l{6}='V101'; l{7}='V102'; l{8}='V103'; l{9}='V201'; l{10}='V202'; l{11}='V203';
% legend(h,l, 'Orientation', 'horizontal', 'Location', 'northoutside');
modes = {['\begin{tabular}{c} ORB-VO \\(', num2str(nMapPoints_orb_VO_mean),  ')\end{tabular}'], ...
    ['\begin{tabular}{c} ORB-SLAM \\(', num2str(nMapPoints_orb_SLAM_mean),  ')\end{tabular}'], ...
    ['\begin{tabular}{c} Ours-VO \\(', num2str(nMapPoints_our_VO_mean),  ')\end{tabular}'], ...
    ['\begin{tabular}{c} Ours-SLAM \\(', num2str(nMapPoints_our_SLAM_mean),  ')\end{tabular}']};
set(gca,'xticklabel',modes, 'TickLength',[0 0], 'ygrid', 'on', 'TickLabelInterpreter', 'latex')
set(gca,'TickLength',[0 0])
ylabel('Map Points')
set(gca, 'Box', 'off', 'GridLineStyle',':', 'BoxStyle', 'full', 'LineWidth', boxlinewidth,'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
ylim([0 max(max(nMapPoints_total))*1.1])


%%
nKeyframes_difference_VO = (nKeyframes_our_VO-nKeyframes_orb_VO_full)./nKeyframes_orb_VO_full*100;
nKeyframes_difference_SLAM = (nKeyframes_our_SLAM-nKeyframes_orb_SLAM_full)./nKeyframes_orb_SLAM_full*100;
nKeyframes_difference = [nKeyframes_difference_VO;nKeyframes_difference_SLAM]';
figure;
subplot(2,7,[1 2 3 4 5 6])
h=bar(nKeyframes_difference);
ylim([-60,20])
xlim([0.5 11.5])
l = cell(1,2);
l{1}='VO'; l{2}='SLAM'; 
legend(h,l, 'Orientation', 'horizontal');
modes = {'M1', '', '', '', '', 'V11', '', '', 'V21', '', ''};
set(gca,'xticklabel',modes, 'TickLength',[0.01 0], 'ygrid', 'on', 'TickLabelInterpreter', 'latex')
% set (gca,'YDir','reverse')
ylabel('Keyframes drop (%)')
set(gca, 'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
average_difference_nkeyframes_VO = mean(nKeyframes_difference_VO);
average_difference_nkeyframes_SLAM = mean(nKeyframes_difference_SLAM);
max_difference_nkeyframes_VO = min(min(nKeyframes_difference_VO));
max_difference_nkeyframes_SLAM = min(min(nKeyframes_difference_SLAM));
disp(['Our VO has average ', num2str(average_difference_nkeyframes_VO), ...
    '% (up to ', num2str(max_difference_nkeyframes_VO) '%) keyframes compared to ORB-VO.' ])
disp(['Our SLAM has average ', num2str(average_difference_nkeyframes_SLAM), ...
    '% (up to ', num2str(max_difference_nkeyframes_SLAM) '%) keyframes compared to ORB-SLAM.' ])

nMapPoints_difference_VO = (nMapPoints_our_VO-nMapPoints_orb_VO_full)./nMapPoints_orb_VO_full*100;
nMapPoints_difference_SLAM = (nMapPoints_our_SLAM-nMapPoints_orb_SLAM_full)./nMapPoints_orb_SLAM_full*100;
nMapPoints_difference = [nMapPoints_difference_VO;nMapPoints_difference_SLAM]';
subplot(2,7,[8 9 10 11 12 13])
h=bar(nMapPoints_difference);
ylim([-60,30])
xlim([0.5 11.5])
l = cell(1,2);
l{1}='VO'; l{2}='SLAM'; 
legend(h,l, 'Orientation', 'horizontal');
modes = {'M1', '', '', '', '', 'V11', '', '', 'V21', '', ''};
set(gca,'xticklabel',modes, 'TickLength',[0.01 0], 'ygrid', 'on', 'TickLabelInterpreter', 'latex')
% set (gca,'YDir','reverse')
ylabel('Map points drop (%)')
set(gca, 'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
average_difference_nMapPoints_VO = mean(nMapPoints_difference_VO);
average_difference_nMapPoints_SLAM = mean(nMapPoints_difference_SLAM);
max_difference_nMapPoints_VO = min(min(nMapPoints_difference_VO));
max_difference_nMapPoints_SLAM = min(min(nMapPoints_difference_SLAM));
disp(['Our VO has average ', num2str(average_difference_nMapPoints_VO), ...
    '% (up to ', num2str(max_difference_nMapPoints_VO) '%) map points compared to ORB-VO.' ])
disp(['Our SLAM has average ', num2str(average_difference_nMapPoints_SLAM), ...
    '% (up to ', num2str(max_difference_nMapPoints_SLAM) '%) keyframes compared to ORB-SLAM.' ])

%%
plot_factor = plot_factor_original;
linewidth = linewidth_original;
markerlinewidth = markerlinewidth_original;
boxlinewidth = boxlinewidth_original;
fontsize = fontsize_original;