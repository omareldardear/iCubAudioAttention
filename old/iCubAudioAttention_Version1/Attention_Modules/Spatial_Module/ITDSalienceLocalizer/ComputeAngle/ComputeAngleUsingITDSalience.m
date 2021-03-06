function [angle,tempLag,lagSpace,trigger,visFrame] = ComputeAngleUsingITDSalience(S,background)
% takes S a stereo sound vector (chans x samples) 
%returns an angle to a peak in GCC_PHAT, the vector of lags, and the
%difference of lags between current frame and previous frame


global P;

%compute GCC-PHAT

%define the width of the window over which to inspect lags (we're only
%interested in lags of a few 10's- the rest would be reverberation


%xplode the array to make this more clear
s1=S(1,:);
s2=S(2,:);

f1=fft(s1);
f2=fft(s2);



%compute the weighted generalized correlation coefficient
gcc_phat=(f1.*conj(f2))./abs(f1.*conj(f2));

%compute the inverse transform
gcc_inv = ifft(gcc_phat);

%make it real
gcc_inv_real=real(gcc_inv);
%gcc_inv_real=gcc_inv;

%shift it to zero the 0th lag
gcc_inv_shifted=fftshift(gcc_inv_real);


%this could be made *much faster* by only working on the subregion of the
%vectors that correspond to useful angles...in the future

% if(isnan(background))
%     background=gcc_inv_shifted; %if we're given NaN then we know the previous frame was triggered.  We want to suppress this one.
% end

%frame's lag space
lag_dif=(gcc_inv_shifted-background);
lag_dif(lag_dif<0)=0;  %ignore offsets for now


%find the 0th lag
middle=floor(length(lag_dif)/2+2);

%find the index with the biggest peak
if(isnan(P.attentionBeamBounds(1))) %no bounds were specified so use the entire window
    
    
    %display(['computing angle for lag ' num2str(lag)]);
    
    %%%%%%%%%
    %these lines will just find the biggest peak
    wVec=[P.weightsV flipdim(P.weightsV,2)];
    window=lag_dif(middle-P.ITDWindow/2:middle+P.ITDWindow/2-1);
    window=window.*wVec;
    [peak,tempLag] = max(window); %only the window  lags are relevant
    lag=tempLag-P.ITDWindow/2;
    
else %bounds were specified, so only look within a range of lags
    [peak,~] = max(lag_dif(middle+P.attentionBeamBounds(1):middle+P.attentionBeamBounds(2)));
    lag=floor(mean(P.attentionBeamBounds)); %you've already specified an angle to watch for transients
    display(['peak value within the attention beam was ' num2str(peak)]);
end

%employ a simple trigger
if peak>P.GCCPeakThreshold
    trigger=1;
    if(~isnan(P.attentionBeamBounds(1)))%report that we know the speaker is in the beam
        display('I know you are there!');
    end
else
    trigger=0;
end

%convert the lag in samples to lag in seconds
angle=ConvertLagToAngle(lag);

%display(['angle = ' num2str(angle)]);

%send this frame's gcc-phat vector back to be passed along to this function during the next frame
lagSpace=gcc_inv_shifted;

% % % % %%%%%check your work
% hold off;
% %pl=plot(linspace(-P.ITDWindow/2,P.ITDWindow/2,length(lag_dif(middle-P.ITDWindow/2:middle+P.ITDWindow/2))),lag_dif(middle-P.ITDWindow/2:middle+P.ITDWindow/2));
% pl=plot(linspace(-P.ITDWindow/2,P.ITDWindow/2,length(lag_dif(middle-P.ITDWindow/2:middle+P.ITDWindow/2))),gcc_inv_shifted(middle-P.ITDWindow/2:middle+P.ITDWindow/2));
% set(pl,'Color','red','LineWidth',2);
% 
% hold on;
% %plot(background(middle-P.ITDWindow/2:middle+P.ITDWindow/2),'r');
% %plot(windowR,'r');
% ylim([0 .2]);
% drawnow;

%return the data you want to visualize:
visFrame=lag_dif(middle-floor(P.ITDWindow/2):middle+ceil(P.ITDWindow/2));

return

end

