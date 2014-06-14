function [didAddObject]=AddNewObject(OStruct)
%ADDNEWOBJECT
%takes a new object struct and adds it to the object stack
%default behaviour should be that new objects get automatically selected
%and go on the top of the stack

%AddNewObject must decide whether to integrate (merge with other recent
%objects on the stack) or substitute (displace other objects, namely the
%one at the top of the stack...masking)

%substitution is not implemented yet

maxIntegrationTime=0.550; %seconds, upper time bound on whether two objects are coincident enough to be called the same object
integrationTime_nanos=uint64(maxIntegrationTime*1000000000.0);


[objFileMap,numObjectsMap,numObjects,isBusyMap]=MapObjectFile;

while(isBusyMap.Data(1,1).isBusy==1)
    %the object stack is being written by another process, so block here
    display('waiting patiently for my turn to modify the object stack');
end

isBusyMap.Data(1,1).isBusy=1;

%prevent distraction...there is a lot to do here but for now just prevent
%displacing selected objects...in the future this should be configurable
%and dynamic...right now do strong early selection

if(objFileMap.Data(1,1).isSelected==0 || OStruct.isSelected==1) %only take action if the top of there is an unselected object at the top of the stack or if the incoming object is already tagged as selected
    
    %get a list of all the features stored in the objects
    features=fieldnames(OStruct); %size of this cell array tells you how many features
    
    oldTime=objFileMap.Data(1,1).timeStamp;
    newTime=OStruct.timeStamp;
    deltaTime=newTime-oldTime;
    
    oldName=objFileMap.Data(1,1).name;
    newName=cast(OStruct.name,'uint16'); %a bit more voodoo, object files encode name as an array of uint16 chars  
    
    if(isequal(oldName,newName)==0 &&  deltaTime < integrationTime_nanos) %add by integration
        
        display(['integrating ' oldName ' with ' newName ' but comparing yields ' num2str(strcmp(oldName,newName))]);
        
        %rename this object to indicate it's been merged with the previous top
        %object
        OStruct.name='mergedxx';
        OStruct.isSelected=1; %automatically select coincident events
        
        %integrate:  this is tricky, policies need to be set up for how to
        %handle different situations, extend this
        
        if(isnan(OStruct.onsetAzimuth))
            OStruct.onsetAzimuth=objFileMap.Data(1,1).onsetAzimuth; %if we aren't provided an azimuth, grab it from previous object (which should be an ITD transient))
        end
        
        %now assign all the fields
        for i=1:length(features) %run through all the feature fields
            if(strcmp(features(i),'name'))
                objFileMap.Data(1,1).(features{i})=cast(OStruct.(features{i}),'uint16'); %a hoop I jumped through so that you can assign with object names as strings...thank me later...-matt
            else
                objFileMap.Data(1,1).(features{i})=OStruct.(features{i});
            end
        end
        
        didAddObject=0; %flag that we integrated an object to the stack
        
    else %add by shifting everything down and adding new object to top of stack
        display(['Adding new object to top of stack at:  ' num2str(newTime) ' with time delta ' num2str(deltaTime)]);
        
        %shift everything down...this is slow and inefficient
        index=numObjects; %work from the bottom of the stack up
        while(index>=1) %note that at initial startup, numObjects will be zero and only the default object will be added below
            
            for i=1:length(features) %run through all the feature fields, this is inefficient but MATLAB won't replace structs in memmaped fields, this could be reworked by directly writing to the file if it needs to be sped up...or you could switch to a pointer system
                
                objFileMap.Data(index+1,1).(features{i})=objFileMap.Data(index,1).(features{i});
                
            end
            
            index=index-1; %work up the stack from the bottom (to do: trap an error due to running off the bottom of the stack if you add too many objects)
            
        end
        
        %now add the new object at the top
        
        for i=1:length(features) %run through all the feature fields
            if(strcmp(features{i},'name'))
                objFileMap.Data(1,1).(features{i})=cast(OStruct.(features{i}),'uint16'); %a hoop I jumped through so that you can assign with object names as strings...thank me later...-matt
            else
                objFileMap.Data(1,1).(features{i})=OStruct.(features{i});
            end
        end
        
        %increment the number of objects on the stack
        numObjectsMap.Data(1,1).numObjects=numObjects+1;
        didAddObject=1; %flag that we added an object to the stack
        
        
        
    end
else
    didAddObject=0;
end

isBusyMap.Data(1,1).isBusy=0;

end








