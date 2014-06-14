
%SendAudio monitors the object stack.  When a selected object
%appears at the top of the stack, SendAudio will optionaly process the signal and send the signal out the line-out port using
%Psychophysics Toolbox Portaudio interface




%load parameters
LoadParameters;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%memory map the incoming audio data dump file
%and get the audio stream queueueued up
%the fixed lag is important: since the run loop below runs through small
%frames as fast as PsychPortAudio fills them into the output buffer, it
%doesn't have the same time scale as other modules.  Since other modules
%are working over hundreds of ms, this module has to "look back in time" to
%recover relevant audio data.  How far back?  It should at least account
%for the difference in frame sizes between the attention orienting modules
%and the frames sent to PsychPortAudio's buffer (e.g. 10240 vs. 1024 so it
%should look back 10 frames).  But also account fixed lag in those modules!

[mIn,sampleD]=OpenAudioInputData;
tempMostRecentSample=sampleD.Data(1,1).f;
readIndex = tempMostRecentSample - (P.kFrameSize_samples - 1) - P.kFixedLag_samples;  %set the read index to the first sample in the next frame to be read...here we need to be careful to lag the audio data dump by some small by non-trivial time

pre=zeros(2,P.kFrameSize_samples*2);%make a vector to preload into the buffer
silence=zeros(2,P.kFrameSize_samples);%send this when we don't want to send audio
InitializePsychSound(1);%set up audio output
paoutput = PsychPortAudio('Open', [], 1, 2, 48000, 2,1024);

%get a frame ready
audiodata=mIn.Data(1,1).d(:,readIndex:readIndex+P.kFrameSize_samples-1); %the first frame is ready

scaleFactor=1/2^15;  %precompute for speed
audiodata=double(audiodata); %scale the audio from 16bit signed ints to .wav
audiodata=audiodata.*scaleFactor;

%start playback
PsychPortAudio('FillBuffer', paoutput, pre);
playbackstart = PsychPortAudio('Start', paoutput, 0,0,1);

%%%%%%%%
%done setting up audio
%%%%%%%%


%%%%%%%%%%%%%%%%%%%%
%memory map the object stack
[objFileMap,~,~,isBusyMap]=MapObjectFile;


%%%%%%%%%%%%
%loop
done=0;
frameT=tic;
while(~done)
    %clc;
    %first immediately fill the buffer with the next frame
    %if the object is selected
    display(['that frame took ' num2str(toc(frameT)) ' seconds']);
    frameT=tic;
    if(objFileMap.Data(1,1).isSelected==1  ) %there is an attended object on the stack
        
        display('sending audio');
        %%%%%Simple selection of channels
%         SelectChannels will clean up the signal by ignoring one of the
%         microphones
        audiodata=SelectChannels(audiodata,objFileMap.Data(1,1).onsetAzimuth);
        %%%%%%%%%
        
        %%%%Using ICA
        %use EEGLab runica() to seperate the sources
        %audioData=SeperateSources(audiodata,objFileMap.Data(1,1).onsetAzimuth);
        
       % audiodata=FilterWithVoicebox(audiodata,P.kSampleRate);
        
        %in case you need to adjust the gain...something might be flaky
        %with the 16-bit audio data
        audiodata=audiodata*P.kGain;
        
        
        %         subplot(2,1,1);
        %         plot(audiodata(1,:));
        %         ylim([-.1 .1]);
        %         subplot(2,1,2);
        %         plot(audiodata(2,:));
        %         ylim([-.1 .1]);
        %         drawnow;
        
        %send the signal OUT
        [underflow,~,~]=PsychPortAudio('FillBuffer', paoutput, audiodata,1);
        
        
    else
        [underflow,~,~]=PsychPortAudio('FillBuffer', paoutput, silence,1); %else send zeros, how's that for early selection!
        %display('sssshhhhh');
    end
    
    %prepare next frame
    readIndex=readIndex+P.kFrameSize_samples;
    audiodata=mIn.Data(1,1).d(:,readIndex:readIndex+P.kFrameSize_samples-1); 

    %scale the audio from signed 16-bit ints to doubles between -1 and 1
    audiodata=double(audiodata).*scaleFactor;
   
    
   %keep looping fast, the sound driver will take care of the rest
   %ICLUDING TIME because PsychPortAudio will wait until the frame fills
   %into the buffer....note that this means the run loop here will not have
   %the same rate as other loops
    
    
    
end