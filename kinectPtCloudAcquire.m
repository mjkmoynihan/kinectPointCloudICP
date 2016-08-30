function [LabSceneData,colorFrameData,depthFrameData, trackingCoords,isTracked]...
    = kinectPtCloudAcquire(numFrames, framesPerTrigger)
% KINECTPTCLOUDACQUIRE Record color and depth scene data. Extract point
% cloud information per frame and return point cloud sequence. 
    
    
% Check that numFrames and framesPerTrigger are divisible.
    while mod(numFrames,framesPerTrigger) ~=0
        message = strcat('numFrames is: ',num2str(numFrames),...
            '. Please enter framesPerTrigger such that numFrames is a multiple :');
        prompt = input(message);
        framesPerTrigger = prompt;
    end
        
% Create the VIDEOINPUT objects for the two streams
    message = 'Creating Video Input Devices ';
    disp(message); 
    tic;
    colorDevice = videoinput('kinect',1); 
    depthDevice = videoinput('kinect',2);
    toc
    
% Set Tracking mode on and store metadata
    depthSrc = getselectedsource(depthDevice);
    depthSrc.TrackingMode = 'Skeleton';

    % Set the triggering mode to 'manual'
    message = 'Setting trigger modes and timeouts ';
    disp(message);
    tic
    triggerconfig([colorDevice depthDevice],'manual');
    colorDevice.TriggerRepeat = ceil((numFrames/framesPerTrigger)-1);
    depthDevice.TriggerRepeat = ceil((numFrames/framesPerTrigger)-1);
   
    
    % Set timeouts to allow for device delay
    set(colorDevice,'Timeout',180);
    set(depthDevice,'Timeout',180); 

    colorDevice.FramesPerTrigger = framesPerTrigger;
    depthDevice.FramesPerTrigger = framesPerTrigger;
    toc
    
    % Set Framerate (Framerate = 30fps/FrameGrabInterval) 
    colorDevice.FrameGrabInterval = 1.5;
    depthDevice.FrameGrabInterval = 1.5;
    
    % Configure the logging mode to disk if desired
    % vidobj.LoggingMode = 'disk';

    % Display Preview
    message = 'Initialising previews ';
    disp(message);
    tic
    rgbPreview = preview(colorDevice); %#ok<NASGU>
    depthPreview = preview(depthDevice); %#ok<NASGU>
    toc

    % Start the color and depth device. This begins acquisition, but does not
    % start logging of acquired data.
    message = 'Readying Acquisition Devices ';
    disp(message); 
    tic
    start([colorDevice depthDevice]);
    toc

    % Trigger the devices to start logging of data.
    message = strcat('Devices ready. Press any key to begin acquisition. ');
    disp(message); 
    beep
    
    % Any key input to being acquisition
    pause;

    message = strcat('Acquisition and logging ',num2str(numFrames),' frames. ');
    disp(message); 
    
    % Initialise cumstom metaData struct
    skeletonData(numFrames).FrameNumber = numFrames;
    
    times = zeros(1,numFrames/framesPerTrigger); 
    
    % Acquire, read, dump buffer structure for every framesPerTrigger
    for triggerCount = 1:1:numFrames/framesPerTrigger
    tic
        message = strcat('Triggered ',num2str(framesPerTrigger),' frames');
        disp(message);
        trigger([colorDevice depthDevice]);

        % Retrieve the acquired data
        message = 'retrieving acuired data ';
        disp(message);  
        
        if ~exist('depthFrameData','var')
            [colorFrameData] = getdata(colorDevice);
            [depthFrameData,~,metaData] = getdata(depthDevice);
            
            for i = 1:size(metaData,1)
                skeletonData(i).FrameNumber = metaData(i).FrameNumber;
                skeletonData(i).IsSkeletonTracked = metaData(i).IsSkeletonTracked;
                skeletonData(i).JointWorldCoordinates = metaData(i).JointWorldCoordinates;
                skeletonData(i).SegmentationData = uint8(metaData(i).SegmentationData);
                
            end
            
            clear metaData
            
        else
           colorFrameData = cat(4,colorFrameData,getdata(colorDevice));
           [depthFrames,~,metaData] = getdata(depthDevice); 
           depthFrameData = cat(4,depthFrameData,depthFrames);
           j  = 1;
           % Awkard indexing, ensures proper creation of skeletonData
           % struct
           for i = triggerCount*framesPerTrigger-framesPerTrigger+1:triggerCount*framesPerTrigger
                skeletonData(i).FrameNumber = metaData(j).FrameNumber;
                skeletonData(i).IsSkeletonTracked = metaData(j).IsSkeletonTracked;
                skeletonData(i).JointWorldCoordinates = metaData(j).JointWorldCoordinates;
                skeletonData(i).SegmentationData = uint8(metaData(j).SegmentationData);
                j = j+1;
                
           end
           
           clear depthFrames metaData; 
           
        end
        

        % Clear Memory
        message = 'Clearing buffer';
        disp(message);
        flushdata(colorDevice);
        flushdata(depthDevice);
        
        % Store processing time
        times(triggerCount)=toc;
    
    end
    
    % Stop the devices
    message = 'Scene recorded. Closing streams and previews ';
    disp(message); 
    tic
    stop([colorDevice depthDevice]); 
    toc
    closepreview(colorDevice);
    closepreview(depthDevice);
    
    
    % Create the data array
    LabSceneData = cell(1,numFrames);
       
    %% Human Tracking and masking. 
    
    % Structuring element for dilation
    se = ones(5); 
    
    % Store coords
    trackingCoords = zeros(numFrames,3,6); 
    
    % initialise logical check if tracking occurs
    isTracked = false(numFrames,1);
    
    % Extract the coordinates of the skeleton head position and ID for each
    % tracked skeleton. Usage is JointWorldCoordinates(JointID,Coords,SkelID)
    for i = 1:length(skeletonData)
        
        for j = 1:6
            if skeletonData(i).IsSkeletonTracked(1,j);
            isTracked(i)=true;
            trackingCoords(i,:,j) = skeletonData(i).JointWorldCoordinates(4,:,j);
            end  
        end
        
        % Generate point clouds with depth centric alignment of RGB and 
        % depth frames. 
        ptCloud = pcfromkinect(depthDevice,depthFrameData(:,:,:,i),...
        colorFrameData(:,:,:,i),'depthCentric');
        LabSceneData{1,i} = ptCloud; 
    
        % If tracking data is available, extract mask, subtract mask from
        % current frame, reconstruction point cloud. 
        if (isTracked(i))

            % Convert mask and dilate to prevent haloing
            mask = im2bw(skeletonData(i).SegmentationData,0.001);
            mask = imdilate(mask,se); 

            depthFrameDataMasked = ...
                depthFrameData(:,:,:,i)-5000*uint16(mask); %5000 = scale factor i.e maximum uppr value

            ptCloudMasked = pcfromkinect(depthDevice, depthFrameDataMasked,...
                colorFrameData(:,:,:,i),'depthCentric');
            LabSceneData{1,i} = ptCloudMasked;
            
            
        end

    end
    
    clearvars -except LabSceneData depthFrameData colorFrameData ...
             trackingCoords isTracked
        
    message = 'SAVE AND OVERWRITE DATASET? YES=1, NO=0 ';
    prompt = input(message);
    
    if prompt == 1
        
        message = 'Dataset Name? (e.g. "<dataset_name>.mat"): ';
        FileName = input(message,'s');
        
        % Specify save version in order to avoid creating a copy of large
        % datasets. 
        save(FileName, 'LabSceneData','depthFrameData','colorFrameData',...
            'trackingCoords','isTracked','-v7.3');
        
        message = 'Dataset saved to file ';
        disp(message);
    end
    
    message = 'Dataset Created. Acquisition finished! ';
    disp(message);


end