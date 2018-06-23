function [ ATE ] = ComputeATE( my_data, gt_data_sync )


[ATE, R, t, scale ] = AlignSimEfficient( gt_data_sync(:,2:4), my_data(:,2:4) );


end


