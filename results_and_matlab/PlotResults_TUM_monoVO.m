%% Initialize:

plot_factor = 0.7;
linewidth = 6*plot_factor;
markerlinewidth = 3*plot_factor;
boxlinewidth = 4*plot_factor;
fontsize = 60*plot_factor;

%% Draw Cumulative Error Plot
figure(999)
clf

[ MallErrorsAlign_RT_default, MallErrorsR_RT_default, MallErrorsS_RT_default, ~, ~, ~, Mn_RT_default, ~] = getSortedLOOPError( DSO_DEFAULT_RESULTS_TUM_monoVO, [0:9] + 1 );
[ MallErrorsAlign_RT_reduced, MallErrorsR_RT_reduced, MallErrorsS_RT_reduced, ~, ~, ~, Mn_RT_reduced, ~] = getSortedLOOPError( DSO_REDUCED_RESULTS_TUM_monoVO, [0:9] + 1 );
[ OallErrorsAlign_RT_vo, OallErrorsR_RT_vo, OallErrorsS_RT_vo, ~, ~, ~, On_RT_vo, ~] = getSortedLOOPError( ORB_VO_RESULTS_TUM_monoVO, [0:9] +1 );
[ OallErrorsAlign_RT_slam, OallErrorsR_RT_slam, OallErrorsS_RT_slam, ~, ~, ~, On_RT_slam, nLC_orb_slam] = getSortedLOOPError( ORB_SLAM_RESULTS_TUM_monoVO, [0:9] +1 );
[ SallErrorsAlign_RT_vo, SallErrorsR_RT_vo, SallErrorsS_RT_vo, ~, ~, ~, Sn_RT_vo, ~] = getSortedLOOPError( OUR_VO_RESULTS_TUM_monoVO, [0:9] +1 );
[ SallErrorsAlign_RT_slam, SallErrorsR_RT_slam, SallErrorsS_RT_slam, ~, ~, ~, Sn_RT_slam, nLC_our_slam] = getSortedLOOPError( OUR_SLAM_RESULTS_TUM_monoVO, [0:9] +1 );


subplot(1, 7,[1 2 3 4])
hold on
plot(SallErrorsAlign_RT_vo, 1:Sn_RT_vo, 'red','LineWidth',linewidth,'LineStyle','-')
hold on
plot(SallErrorsAlign_RT_slam, 1:Sn_RT_slam, 'red','LineWidth',linewidth,'LineStyle','--')
hold on
plot(MallErrorsAlign_RT_default, 1:Mn_RT_default, 'green','LineWidth',linewidth,'LineStyle','-')
hold on
plot(MallErrorsAlign_RT_reduced, 1:Mn_RT_reduced, 'green','LineWidth',linewidth,'LineStyle','--')
hold on
plot(OallErrorsAlign_RT_vo, 1:On_RT_vo, 'blue','LineWidth',linewidth,'LineStyle','-')
hold on
plot(OallErrorsAlign_RT_slam, 1:On_RT_slam, 'blue','LineWidth',linewidth,'LineStyle','--')

legend('Ours VO', 'Ours SLAM','DSO default', 'DSO reduced', 'ORB-VO','ORB-SLAM','Location', 'southeast')
axis([0 10 0 500]);
grid on
set(gca, 'XTick',[0:2:10])
set(gca, 'YTick',[0:100:500])
    set(gca, 'Box', 'off', 'GridLineStyle',':', 'BoxStyle', 'full', 'LineWidth', boxlinewidth,'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
    set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
ylabel('Number of runs')
title('Alignment Error')

%% Process results for color map

DSO_LOOP_default = [DSO_DEFAULT_RESULTS_TUM_monoVO.LOOPerrT(:,1:10)]';
DSO_LOOP_reduced = [DSO_REDUCED_RESULTS_TUM_monoVO.LOOPerrT(:,1:10)]';
DSO_LOOP_default_sorted = sort(DSO_LOOP_default,1);
DSO_LOOP_reduced_sorted = sort(DSO_LOOP_reduced,1);
DSO_LOOP = [DSO_LOOP_default_sorted;DSO_LOOP_reduced_sorted];

ORB_LOOP_vo = [ORB_VO_RESULTS_TUM_monoVO.LOOPerrT(:,1:10)]';
ORB_LOOP_slam = [ORB_SLAM_RESULTS_TUM_monoVO.LOOPerrT(:,1:10)]';
nLoopClosures_orb_slam = [ORB_SLAM_RESULTS_TUM_monoVO.nLoopClosures(:,1:10)]';
ORB_LOOP_vo_sorted = sort(ORB_LOOP_vo,1);
[ORB_LOOP_slam_sorted, orb_idx] = sort(ORB_LOOP_slam,1);
ORB_LOOP = [ORB_LOOP_vo_sorted;ORB_LOOP_slam_sorted];

for col = 1:size(orb_idx,2)
    nLoopClosures_orb_slam(:,col) = nLoopClosures_orb_slam(orb_idx(:,col),col);
end

nLoopClosures_orb_slam = [zeros(size(ORB_LOOP_vo_sorted));nLoopClosures_orb_slam];

OUR_LOOP_vo = [OUR_VO_RESULTS_TUM_monoVO.LOOPerrT(:,1:10)]';
OUR_LOOP_slam = [OUR_SLAM_RESULTS_TUM_monoVO.LOOPerrT(:,1:10)]';
nLoopClosures_our_slam = [OUR_SLAM_RESULTS_TUM_monoVO.nLoopClosures(:,1:10)]';
OUR_LOOP_vo_sorted = sort(OUR_LOOP_vo,1);
[OUR_LOOP_slam_sorted, our_idx] = sort(OUR_LOOP_slam,1);
OUR_LOOP = [OUR_LOOP_vo_sorted;OUR_LOOP_slam_sorted];

for col = 1:size(our_idx,2)
    nLoopClosures_our_slam(:,col) = nLoopClosures_our_slam(our_idx(:,col),col);
end

nLoopClosures_our_slam = [zeros(size(OUR_LOOP_slam));nLoopClosures_our_slam];


%% Plot DSO color map:

figure
clf
% imagesc(imresize(DSO_LOOP_default2,20,'nearest') );
imagesc(DSO_LOOP)
caxis([0 10]);
colormap(jet);
hold on
plot([0 51],[10 10]+0.5,'black','LineWidth',5)

set(gca, 'YTick',[5 15])
set(gca, 'YTickLabel',{'Default', 'Reduced'}, 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
set(gca, 'XTick',[5 10 15 20 25 30 35 40 45 50])
set(gca, 'XTickLabel',{ '5', '10', '15', '20', '25', '30', '35', '40','45', '50'}, 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')

colorbar
title('DSO', 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')

%% Plot ORB color map:

figure
clf
% imagesc(imresize(DSO_LOOP_default2,20,'nearest') );
imagesc(ORB_LOOP)
caxis([0 10]);
colormap(jet);
hold on
plot([0 51],[10 10]+0.5,'black','LineWidth',5)
hold on;

for x = 1:size(ORB_LOOP,1)
    for y = 1:size(ORB_LOOP,2)
        if (nLoopClosures_orb_slam(x,y) > 0)
            text(y-0.55,x,'\bf L', 'FontSize', 20, 'FontName', 'Times New Roman')
        end
    end
end

set(gca, 'YTick',[5 15])
set(gca, 'YTickLabel',{'VO', 'SLAM'}, 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
set(gca, 'XTick',[5 10 15 20 25 30 35 40 45 50])
set(gca, 'XTickLabel',{ '5', '10', '15', '20', '25', '30', '35', '40','45', '50'}, 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')

colorbar
title('ORB', 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')

%% Plot our color map:

figure
clf
% imagesc(imresize(DSO_LOOP_default2,20,'nearest') );
imagesc(OUR_LOOP)
caxis([0 10]);
colormap(jet);
hold on
plot([0 51],[10 10]+0.5,'black','LineWidth',5)
hold on;

for x = 1:size(OUR_LOOP,1)
    for y = 1:size(OUR_LOOP,2)
        if (nLoopClosures_our_slam(x,y) > 0)
            text(y-0.55,x,'\bf L', 'FontSize', 20, 'FontName', 'Times New Roman')
        end
    end
end

set(gca, 'YTick',[5 15])
set(gca, 'YTickLabel',{'VO', 'SLAM'}, 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
set(gca, 'XTick',[5 10 15 20 25 30 35 40 45 50])
set(gca, 'XTickLabel',{ '5', '10', '15', '20', '25', '30', '35', '40','45', '50'}, 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')

colorbar
title('Ours', 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')

