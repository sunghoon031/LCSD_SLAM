function [ allErrorsAlign, allErrorsR, allErrorsS, allErrorsRMSE, allErrorsA, allErrorsE, n, allLoopClosures ] = getSortedLOOPError( DATASET, ids, DATASET2 )

k = size(ids,2);
n1 = size(DATASET.LOOPerrR,1);
n2 = 0;


if(nargin > 2)
    n2 = size(DATASET2.LOOPerrR,1);
end
n = n1+n2;
sqs1=1:n1;
sqs2=1:n2;


allErrorsAlign = nan(k*n,1);
allErrorsR = nan(k*n,1);
allErrorsS = nan(k*n,1);
allErrorsRMSE = nan(k*n,1);
allErrorsA = nan(k*n,1);
allErrorsE = nan(k*n,1);
allLoopClosures = zeros(k*n,1);


for i=sqs1
    allErrorsAlign((i-1)*k+(1:k)) = DATASET.LOOPerrT(i,ids);
    allErrorsR((i-1)*k+(1:k)) = DATASET.LOOPerrR(i,ids);
    allErrorsS((i-1)*k+(1:k)) = exp(abs(log(DATASET.LOOPerrS(i,ids))));
    allErrorsRMSE((i-1)*k+(1:k)) = DATASET.LOOPerrRMSE(i,ids);
    allErrorsA((i-1)*k+(1:k)) = DATASET.LOOPabserrA(i,ids);
    allErrorsE((i-1)*k+(1:k)) = DATASET.LOOPabserrE(i,ids);
    allLoopClosures((i-1)*k+(1:k)) = DATASET.nLoopClosures(i,ids);
end


if(nargin > 2)
    for i=sqs2
        allErrorsAlign(k*n1+(i-1)*k+(1:k)) = DATASET2.LOOPerrT(i,ids);
        allErrorsR(k*n1+(i-1)*k+(1:k)) = DATASET2.LOOPerrR(i,ids);
        allErrorsS(k*n1+(i-1)*k+(1:k)) = exp(abs(log(DATASET2.LOOPerrS(i,ids))));
        allErrorsRMSE(k*n1+(i-1)*k+(1:k)) = DATASET2.LOOPerrRMSE(i,ids);
        allErrorsA(k*n1+(i-1)*k+(1:k)) = DATASET2.LOOPabserrA(i,ids);
        allErrorsE(k*n1+(i-1)*k+(1:k)) = DATASET2.LOOPabserrE(i,ids);
        allLoopClosures(k*n1+(i-1)*k+(1:k)) = DATASET2.nLoopClosures(i,ids);
    end
end


idGood = (allErrorsAlign ~= 0);
allErrorsAlign = allErrorsAlign(idGood);
allErrorsR = allErrorsR(idGood);
allErrorsS = allErrorsS(idGood);
allErrorsRMSE = allErrorsRMSE(idGood);
allErrorsA = allErrorsA(idGood);
allErrorsE = allErrorsE(idGood);
allLoopClosures = allLoopClosures(idGood);

[allErrorsAlign, idx] = sort(allErrorsAlign);
allErrorsR = sort(allErrorsR);
allErrorsS = sort(allErrorsS);
allErrorsRMSE = sort(allErrorsRMSE);
allErrorsA = sort(allErrorsA);
allErrorsE = sort(allErrorsE);
allLoopClosures = allLoopClosures(idx);

n=size(allErrorsAlign,1)*size(allErrorsAlign,2);


end

