function [ rmse, R, t, scale ] = AlignSimEfficient( gtPos, estPos )

if (size(gtPos,1) > size(gtPos,2))
    gtPos = gtPos'; % Make it [3xN]
    estPos = estPos'; % Make it [3xN]
end

N = size(estPos,2);

centroid_est = mean(estPos,2);
centroid_gt = mean(gtPos,2);

estPos_mCentroid = estPos - repmat(centroid_est, 1, N);
gtPos_mCentroid = gtPos - repmat(centroid_gt, 1, N);

H = estPos_mCentroid * gtPos_mCentroid'; %[3xN]*[Nx3] = [3x3]

[U,S,V] = svd(H);
R = V*U';
if det(R) < 0
    V(:,3) = -V(:,3);
    R = V*U';
end


den = 0;
num = 0;
for i=1:N
    % Each increment is [3x1]'*[3x1]=[1x1]
    num = num + gtPos_mCentroid(:,i)'*R*estPos_mCentroid(:,i);
    den = den + estPos_mCentroid(:,i)'*estPos_mCentroid(:,i);
end

scale = num/den;
t = centroid_gt -scale*R*(centroid_est);
rmse =  (sum(sum((scale*R*estPos_mCentroid-gtPos_mCentroid).^2))/N).^0.5;

if(isnan(scale))
    R = nan(3);
end


end

