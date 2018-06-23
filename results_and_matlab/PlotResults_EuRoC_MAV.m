%% Initialize


plot_factor = 1.5;
linewidth = 4*plot_factor;
markerlinewidth = 3*plot_factor;
boxlinewidth = 3*plot_factor;
fontsize = 40*plot_factor;
%% Process

AggregateResults
PlotBarGraphs
% return;


ATE_total_full = [];
ATE_total_half = [];

ATE_total_top_bottom_full_VO_SLAM = [];
nLoopClosures_top_bottom_full_VO_SLAM = [];

for run =1:6  % 1: our vo/slam, 2: dso default, 3: dso reduced, 4: orb_vo 5: orb_slam 6: orb full
    if run == 1 
        ATE_total_full = ATEs_our_VO;
        ATE_total_half = ATEs_our_SLAM;
        
        [ATEs_our_VO, vo_idx] = sort(ATEs_our_VO,1);
        [ATEs_our_SLAM, slam_idx] = sort(ATEs_our_SLAM,1);
        
        ATE_total_top_bottom= [ATEs_our_VO; ATEs_our_SLAM];

        
        for col = 1:size(slam_idx,2)
            nLoopClosures_our_VO(:,col) = nLoopClosures_our_VO(vo_idx(:,col),col);
            nLoopClosures_our_SLAM(:,col) = nLoopClosures_our_SLAM(slam_idx(:,col),col);
        end

        nLoopClosures_top_bottom = [nLoopClosures_our_VO; nLoopClosures_our_SLAM];
        
        figure;
        
        imagesc(ATE_total_top_bottom)
        hold on;
        for x = 1:size(ATE_total_top_bottom,1)
            for y = 1:size(ATE_total_top_bottom,2)
                if (nLoopClosures_top_bottom(x,y) > 0)
                    text(y-0.3,x,'\bf L', 'FontSize', 30, 'FontName', 'Times New Roman')
                end
            end
        end
        
        
    elseif run == 2
        ATE_total_full = ATEs_dso_default_full;
        ATE_total_half = ATEs_dso_default_half;
        
    elseif run == 3
        ATE_total_full = ATEs_dso_reduced_full;
        ATE_total_half = ATEs_dso_reduced_half;
        
        ATEs_dso_default_full = sort(ATEs_dso_default_full,1);
        ATEs_dso_reduced_half = sort(ATEs_dso_reduced_half,1);
        
        ATE_total_top_bottom= [ATEs_dso_default_full; ATEs_dso_reduced_half];
        figure;
        imagesc(ATE_total_top_bottom)

    elseif run == 4
        ATE_total_full = ATEs_orb_VO_full;
        ATE_total_half = ATEs_orb_VO_half;
        
        [ATE_total_full, full_idx] = sort(ATE_total_full,1);
        [ATE_total_half, half_idx] = sort(ATE_total_half,1);
        
        ATE_total_top_bottom= [ATE_total_full; ATE_total_half];
        figure;
        imagesc(ATE_total_top_bottom)
    elseif run==5
        
        ATE_total_full = ATEs_orb_SLAM_full;
        ATE_total_half = ATEs_orb_SLAM_half;
        [ATE_total_full, full_idx] = sort(ATE_total_full,1);
        [ATE_total_half, half_idx] = sort(ATE_total_half,1);
        ATE_total_top_bottom_full_VO_SLAM = [ATE_total_top_bottom(1:size(ATE_total_top_bottom,1)/2,:); ATE_total_full];
        ATE_total_top_bottom= [ATE_total_full; ATE_total_half];
        
        for col = 1:size(full_idx,2)
            nLoopClosures_orb_SLAM_full(:,col) = nLoopClosures_orb_SLAM_full(full_idx(:,col),col);
            nLoopClosures_orb_SLAM_half(:,col) = nLoopClosures_orb_SLAM_half(half_idx(:,col),col);
        end
        nLoopClosures_top_bottom_full_VO_SLAM = [zeros(size(nLoopClosures_orb_SLAM_full)); nLoopClosures_orb_SLAM_full];
        nLoopClosures_top_bottom = [nLoopClosures_orb_SLAM_full; nLoopClosures_orb_SLAM_half];
        
        figure;
        imagesc(ATE_total_top_bottom)
        hold on;
        for x = 1:size(ATE_total_top_bottom,1)
            for y = 1:size(ATE_total_top_bottom,2)
                if (nLoopClosures_top_bottom(x,y) > 0)
                    text(y-0.3,x,'\bf L', 'FontSize', 30, 'FontName', 'Times New Roman')
                end
            end
        end
    elseif run == 6
        figure;

        imagesc(ATE_total_top_bottom_full_VO_SLAM)
        hold on;
        for x = 1:size(ATE_total_top_bottom_full_VO_SLAM,1)
            for y = 1:size(ATE_total_top_bottom_full_VO_SLAM,2)
                if (nLoopClosures_top_bottom_full_VO_SLAM(x,y) > 0)
                    text(y-0.3,x,'\bf L', 'FontSize', 30, 'FontName', 'Times New Roman')
                end
            end
        end

        yt1 = 'VO';
        yt2 = 'SLAM';
        set(gca,'Ytick', [5.5, 15.5],'Yticklabel',{yt1, yt2}, 'FontName', 'Times New Roman')
        caxis([0 0.5])
        colormap(jet);
        cbr = colorbar;
        set(cbr, 'YTick', 0:0.1:0.5, 'FontSize', fontsize, 'FontName', 'Times New Roman', 'FontWeight','normal')
        
    end


    %% Lines, modes and titles in each color map: 
    if (run ~= 2)

        line([0,22.5],[10.5,10.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([2.5,2.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([4.5,4.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([6.5,6.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([8.5,8.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([10.5,10.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([12.5,12.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([14.5,14.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([16.5,16.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([18.5,18.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        line([20.5,20.5],[0 20.5], 'Color', 'black',  'LineWidth', 3); hold on;
        set(gca,'Xtick', 1.5:2:21.5,'Xticklabel',{'M1', 'M2','M3', 'M4', 'M5', 'V11', 'V12', 'V13', 'V21', 'V22', 'V23'}, 'FontName', 'Times New Roman', 'Fontsize', 35)

        
        if (run == 1 || run == 6)
            yt1 = 'VO';
            yt2 = 'SLAM';
        else
            yt1 = 'Default';
             yt2 = 'Reduced';
        end
        
        set(gca, 'Ytick', [5.5, 15.5],'Yticklabel',{yt1, yt2}, 'TickLabelInterpreter', 'latex', 'FontName', 'Times New Roman')
        caxis([0 0.5])
        colormap(jet);
        cbr = colorbar;
        set(cbr, 'YTick', 0:0.1:0.5, 'FontSize', fontsize, 'FontName', 'Times New Roman', 'FontWeight','normal')

        if run == 1
            title('Ours', 'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
        elseif run == 3
            title('DSO',  'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
        elseif run == 4
            title('ORB-SLAM w/o Loop Closure',  'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
        elseif run == 5
            title('ORB-SLAM with Loop Closure',  'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
        elseif run == 6
            title('ORB',  'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
        end
    end
        

    %% Cumulative Error Plot
    
    % (1) Process the data!
    ATE_total_full_1d = ATE_total_full(:);
    ATE_total_half_1d = ATE_total_half(:);
    error_bins = 0:0.01:0.5;
    frequency_full = zeros(1, length(error_bins));
    frequency_half = zeros(1, length(error_bins));
    
    for i = 1:length(error_bins)
        for j = 1:length(ATE_total_full_1d)
            if (ATE_total_full_1d(j)<= error_bins(i))
                frequency_full(i) = frequency_full(i) + 1;
            end
        end
    end

     for i = 1:length(error_bins)
        for j = 1:length(ATE_total_half_1d)
            if (ATE_total_half_1d(j)<= error_bins(i))
                frequency_half(i) = frequency_half(i) + 1;
            end
        end
     end

     for i = 1:length(error_bins)-1
         if (frequency_full(i+1) == 0)
             frequency_full(i) = nan;
         end
          if (frequency_half(i+1) == 0)
             frequency_half(i) = nan;
          end
     end
    
    
    % (2) Plot the data!
    figure(99)
    hold on;
    subplot(1, 7,[1 2 3 4])
    if run == 1
        plot(error_bins, frequency_full,  'r-', 'LineWidth', linewidth);
        hold on;
        plot(error_bins, frequency_half,  'r--', 'LineWidth', linewidth);
    elseif run == 2
        plot(error_bins, frequency_full,  'g-', 'LineWidth', linewidth);
    elseif run == 3
        plot(error_bins, frequency_half, 'g--', 'LineWidth', linewidth);
    elseif run == 4
        plot(error_bins, frequency_full,  'b-', 'LineWidth', linewidth);
    elseif run == 5
        plot(error_bins, frequency_full,  'b--', 'LineWidth', linewidth);

        legend_cell = cell(6,1);
        legend_cell{1} = ' Ours VO';
        legend_cell{2} = ' Ours SLAM';
        legend_cell{3} = ' DSO default';
        legend_cell{4} = ' DSO reduced';
        legend_cell{5} = ' ORB-VO';
        legend_cell{6} = ' ORB-SLAM';
        legend(legend_cell);
    end


    grid on;

    set(gca, 'Ytick', 0:50:200, 'Xtick', 0:0.1:0.5)
    axis([0 0.5 0 220])
    ylabel('number of runs',  'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
    %xlabel('$e_\textrm{rmse}$', 'interpreter', 'latex',  'FontName', 'Times New Roman', 'FontSize', fontsize, 'FontWeight', 'normal')
    title('ATE')
    
    
    
    set(gca, 'Box', 'off', 'GridLineStyle',':', 'BoxStyle', 'full', 'LineWidth', boxlinewidth,'FontSize', fontsize, 'FontWeight', 'normal', 'FontName', 'Times New Roman')
    set(findall(gcf,'type','text'),'FontSize', fontsize, 'FontWeight', 'normal','FontName', 'Times New Roman')
    
   
end
