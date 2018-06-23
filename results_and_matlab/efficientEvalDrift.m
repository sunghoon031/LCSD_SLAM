function [rmse, errTrafo, errAlign, errR, errS, abserrA, abserrE, nkf, nLoopClosures] = efficientEvalDrift( benchmark, sequence, plotfig, run )

errAlign=inf;
errR=inf;
errS=inf;
abserrA=inf;
abserrE=inf;
rmse=inf;
errTrafo=inf(4);
nkf=0;
nLoopClosures=0;



mocapRaw = sequence.mocapRaw;
lsdOpt = importdata([benchmark]);


if(size(lsdOpt,1)==0)
    disp(['NO DATAA ' sequence.name '-' num2str(run)])
    return
end

%Seong Addition:
if (size(lsdOpt,2)==17)
%     if (lsdOpt(1,15)-lsdOpt(1,14) < 0.8*(lsdOpt(1,17)-lsdOpt(1,16)))
%         disp(['Less than 80% covered.. Fail! ' sequence.name '-' num2str(run)])
%         return
%     end
elseif (size(lsdOpt,2)==19)
%     if (lsdOpt(1,17)-lsdOpt(1,16) < 0.8*(lsdOpt(1,19)-lsdOpt(1,18)))
%         disp(['Less than 80% covered.. Fail! ' sequence.name '-' num2str(run)])
%         return
%     end
    nLoopClosures = lsdOpt(1,15);
elseif (size(lsdOpt,2)==10)
    nLoopClosures = lsdOpt(1,10);
end

if(abs(mocapRaw(1,1) - lsdOpt(1,1)) > 1000)
    lsdOpt(:,1) = 2e9-lsdOpt(:,1);
end



[A B] = sort(lsdOpt(:,1));
lsdOpt = lsdOpt(B,:);


if(nargin < 3 )
    plotfig=0;
end

if sum(sum(isnan(lsdOpt))) > 0
    ['IS NAN' sequence.name]
    return
end

lsdPos = lsdOpt(:,2:4);
lsdTme = lsdOpt(:,1);
nkf = size(lsdTme,1);

% assiciate.
gtPos = zeros(size(lsdTme,1),3);
gtID = 1;
for i=1:size(lsdTme,1)
    while(lsdTme(i) - mocapRaw(gtID,1) > 0.001)
        gtID = gtID+1;
    end
    
    if(abs(lsdTme(i) - mocapRaw(gtID,1)) > 0.001)
        disp('ERROR, cannot associate frame well')
    end
    gtPos(i,1:3) = mocapRaw(gtID,2:4);
end

nframes = size(mocapRaw,1);
timesAlign =  [mocapRaw(1,1) mocapRaw(floor(nframes/2),1)];
timesEval =  [mocapRaw(ceil(nframes/2),1) mocapRaw(nframes,1)];


% align start segment
lsdFramesAlign = (lsdTme >= timesAlign(1)) & (lsdTme <= timesAlign(2)) & (~isnan(gtPos(:,1)));
lsdPosAlign = lsdPos(lsdFramesAlign,:);
gtPosAlign = gtPos(lsdFramesAlign,:);
%lsdTmeAlign = lsdTme(lsdFramesAlign,:);


lsdFramesEval = (lsdTme >= timesEval(1)) & (lsdTme <= timesEval(2)) & (~isnan(gtPos(:,1)));
lsdPosEval = lsdPos(lsdFramesEval,:);
gtPosEval = gtPos(lsdFramesEval,:);
%lsdTmeEval = lsdTme(lsdFramesEval,:);

gtAvailableStartSegment_startTime = 0;
gtAvailableStartSegment_endTime = 0;
gtAvailableEndSegment_startTime = 0;
gtAvailableEndSegment_endTime = 0;

for i = 1:round(size(mocapRaw,1)/2)
    if (gtAvailableStartSegment_startTime == 0 && ~isnan(mocapRaw(i,end)))
        gtAvailableStartSegment_startTime = mocapRaw(i,1);
    end
    
    if (gtAvailableStartSegment_startTime > 0 && ~isnan(mocapRaw(i,end)))
        gtAvailableStartSegment_endTime = mocapRaw(i,1);
    end
end

for i = round(size(mocapRaw,1)/2):size(mocapRaw,1)
    if (gtAvailableEndSegment_startTime == 0 && ~isnan(mocapRaw(i,end)))
        gtAvailableEndSegment_startTime = mocapRaw(i,1);
    end
    
    if (gtAvailableEndSegment_startTime > 0 && ~isnan(mocapRaw(i,end)))
        gtAvailableEndSegment_endTime = mocapRaw(i,1);
    end
end


% Seong addition: If no ground truth data exists either due to late init or
% early tracking failure:
unfair = false;
if (size(lsdPosAlign,1) < 4 || size(lsdPosEval,1) < 4)
    if (size(lsdPosAlign,1) < 4 && lsdTme(1) < gtAvailableStartSegment_endTime && lsdTme(end) > gtAvailableEndSegment_startTime )
         disp(['[Unfair] Late Init ' num2str(size(lsdPosAlign,1)) ', ' sequence.name '-' num2str(run)])
         unfair = true;
    end
    
    if (size(lsdPosEval,1) < 4 && lsdTme(1) < gtAvailableStartSegment_endTime && lsdTme(end) > gtAvailableEndSegment_startTime)
        disp(['[Unfair] Early Fail ' num2str(size(lsdPosEval,1)) ', ' sequence.name '-' num2str(run)])
        unfair = true;
    end
    
%     if (lsdTme(1) > gtAvailableStartSegment_endTime)
%         disp(['[Fair] Late init ' num2str(size(lsdPosAlign,1)) ', ' sequence.name '-' num2str(run)])
%     end
%     
%     if (lsdTme(end) < gtAvailableEndSegment_startTime)
%         disp(['[Fair] Early Fail' num2str(size(lsdPosEval,1)) ', ' sequence.name '-' num2str(run)])
%     end
%     errAlign = 0.0001;
    return;
end

% align (7DoF)
if(size(lsdFramesEval,1)==0 || size(lsdFramesAlign,1)==0)
    disp(['IS INCOMPLETE ' sequence.name])
    return
end

[ abserrE, RE, tE, scaleE ] = AlignSimEfficient( gtPosEval, lsdPosEval );
[ abserrA, RA, tA, scaleA ] = AlignSimEfficient( gtPosAlign, lsdPosAlign );




if(isnan(abserrE) || isnan(abserrA) || isnan(scaleE) || isnan(scaleA))
    if (isnan(abserrE))
        disp('isnan(abserrE)')
    end
    if (isnan(abserrA))
        disp('isnan(abserrA)')
    end
    if (isnan(scaleE))
        disp('isnan(scaleE)')
    end
    if (isnan(scaleA))
        disp('isnan(scaleA)')
    end

    disp('Something wrong')
    return
end





% get sequence aligned by EVAL.
% size(lsdPos)
% size(RE)
% size(tE)
lsdPosE_aligned = scaleE * lsdPos * RE' + repmat(tE', size(lsdPos,1), 1);

% get sequence aligned by ALIGN.
lsdPosA_aligned = lsdPos * scaleA * RA' + repmat(tA', size(lsdPos,1), 1);

errS = scaleA/scaleE;  % as factor
errorquat = dcm2quat( RE*RA');
errR = 2*acos(errorquat(1))*180/pi;      % as degree
errAlign = (sum(sum((lsdPosE_aligned-lsdPosA_aligned).^2)) / size(lsdPos,1))^0.5;

% Seong commented:
% if( sum(sum(isnan(RE+RA))) > 0)
%     errTrafo=inf(4);
% else
%     errTrafo = [scaleE*RE tE; 0 0 0 1] * [scaleA*RA tA; 0 0 0 1]^-1;      % as degree
% end


[ rmse, ~, ~, ~ ] = AlignSimEfficient( [gtPosAlign; gtPosEval], [lsdPosAlign; lsdPosEval] );





if(plotfig==1)

    clf
    subplot(1,3,1)
    hold on
    plot(lsdTme-lsdTme(1), lsdPosA_aligned,'blue','LineWidth',2);
    plot(lsdTme-lsdTme(1), lsdPosE_aligned,'red','LineWidth',2);
    plot(lsdTme-lsdTme(1), gtPos,'green','LineWidth',3,'LineStyle','--');
    grid on
    axis([0 lsdTme(end)-lsdTme(1) min(min(min(lsdPosE_aligned)),min(min(lsdPosA_aligned)))-4 max(max(max(lsdPosE_aligned)),max(max(lsdPosA_aligned)))+4])

    subplot(1,3,2)
    hold on
    plot(lsdTme-lsdTme(1), lsdPosA_aligned,'blue','LineWidth',2);
    plot(lsdTme-lsdTme(1), lsdPosE_aligned,'red','LineWidth',2);
    plot(lsdTme-lsdTme(1), gtPos,'green','LineWidth',3,'LineStyle','--');
    grid on
    axis([0 lsdTme(end)-lsdTme(1) min(min(min(lsdPosE_aligned)),min(min(lsdPosA_aligned)))-4 max(max(max(lsdPosE_aligned)),max(max(lsdPosA_aligned)))+4])

    subplot(1,3,3)
    hold on
    plot(lsdTme-lsdTme(1), lsdPosA_aligned,'blue','LineWidth',2);
    plot(lsdTme-lsdTme(1), lsdPosE_aligned,'red','LineWidth',2);
    plot(lsdTme-lsdTme(1), gtPos,'green','LineWidth',3,'LineStyle','--');
    grid on
    axis([0 lsdTme(end)-lsdTme(1) min(min(min(lsdPosE_aligned)),min(min(lsdPosA_aligned)))-4 max(max(max(lsdPosE_aligned)),max(max(lsdPosA_aligned)))+4])

end


if(plotfig==2)


H = ([lsdPosE_aligned; lsdPosA_aligned])' * ([lsdPosE_aligned; lsdPosA_aligned]);
[U,S,V] = svd(H);
R = V*U';
if det(R) < 0
    V(:,3) = V(:,3) * -1;
    R = V*U';
end


    lsdPosE_aligned_rot = lsdPosE_aligned*U;
    lsdPosA_aligned_rot = lsdPosA_aligned*U;
    gtPos_rot = gtPos*U;


    d1=2;
    d2=1;

    n=size(gtPos_rot,1);
    clf
    subplot(1,2,1)
    hold on
    plot(gtPos_rot(1:(n/2),d1), gtPos_rot(1:(n/2),d2),'green','LineWidth',3);
    plot(gtPos_rot((n/2):end,d1), gtPos_rot((n/2):end,d2),'green','LineWidth',3);
    plot(lsdPosE_aligned_rot(:,d1), lsdPosE_aligned_rot(:,d2),'red','LineWidth',2)
    plot(lsdPosA_aligned_rot(:,d1), lsdPosA_aligned_rot(:,d2),'blue','LineWidth',2)
    %axis equal
    grid on

        subplot(1,2,2)
    hold on
    plot(gtPos_rot(1:(n/2),d1), gtPos_rot(1:(n/2),d2),'green','LineWidth',3);
    plot(gtPos_rot((n/2):end,d1), gtPos_rot((n/2):end,d2),'green','LineWidth',3);
    plot(lsdPosE_aligned_rot(:,d1), lsdPosE_aligned_rot(:,d2),'red','LineWidth',2)
    plot(lsdPosA_aligned_rot(:,d1), lsdPosA_aligned_rot(:,d2),'blue','LineWidth',2)
   % axis equal
    grid on
end

end

 