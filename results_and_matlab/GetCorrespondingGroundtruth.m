function [ gt_data_sync ] = GetCorrespondingGroundtruth( my_data, gt_data )

my_row = size(my_data,1);
my_col = size(my_data,2);
gt_data_sync = zeros(my_row, my_col);


for i = 1:my_row
    [~, min_idx] = min(abs(gt_data(:,1)-my_data(i,1)));
    gt_data_sync(i,:) = gt_data(min_idx,1:size(gt_data_sync,2));
end

end

