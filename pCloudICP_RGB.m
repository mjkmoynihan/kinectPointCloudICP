clearvars;
close all;

message = 'ACQUIRE? YES=1, NO=0 ';
prompt = input(message);
clear message;

if prompt == 1
    % Acquire
    clear LabSceneData;

    % Specify numFrames for acquisition length.
    % Ideal number of framesPerTrigger tested, found to be either 2 or 5;
    numFrames = 200;
    framesPerTrigger = 50;
    [LabSceneData,colorFrameData,depthFrameData, trackingCoords,isTracked]...
           = kinectPtCloudAcquire(numFrames, framesPerTrigger);

else
    clear prompt;
    % Load data otherwise
    dataFile = fullfile(pwd,'DCU_desk_xyz.mat');
    load(dataFile);
    clear dataFile;

end

% Initialisation Block
% Sequence Crop
initFrame = 1;
finalFrame = 300;
if finalFrame > length(LabSceneData)
    finalFrame = length(LabSceneData);
end

% Enable frame-by-frame point cloud visualiser for debugging
showPtCloudComparison = false;

% Initialise point cloud reference objects (denoise)
ptCloudCurrent  = pcdenoise(LabSceneData{initFrame});

% Downsample to improve speed and accuracy
gridSize = 0.1;
fixed = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
moving = fixed; %initialise

% Initialise Variables
tform = affine3d(eye(4));

ptCloudAligned = ptCloudCurrent;
ptCloudScene = ptCloudCurrent;

mergeSize =  0.015;
droppedFrameCount = 0;
L2dist = zeros(1,finalFrame);

dT(finalFrame) = 0;
dV(finalFrame) = 0;

% Initialise the transformation object that accumulates the transformation.
accumTform = tform;

% Initialise Viewer. Y-axis vertical as specified by Kinect coordinate
% system
figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')

% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

% Initialise camCloud as a pointCloud axes object purely for visualisation.
% camTrack used to store coordinates.

camCloud = zeros(31,3);
camCloud(2:11,1) = 0.01:0.01:0.1;
camCloud(12:21,2) = 0.01:0.01:0.1;
camCloud(22:31,3) = 0.01:0.01:0.1;

camTrack = zeros(finalFrame,7);
camCloud = pointCloud(camCloud);

camColor = zeros(31,3);
camColor(2:11,1) = 255;
camColor(12:21,2) = 255;
camColor(22:31,3) = 255;

camCloud.Color = uint8(camColor);
clear camColor;


% Main Processing Loop

for i = initFrame+1:finalFrame
    
    
    disp('Processing Frame: ');
    disp(num2str(i));
    
    % Line up next frame
    ptCloudCurrent = LabSceneData{1,i};
    
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(LabSceneData{1,i}, 'gridAverage', gridSize);
    
    % SIFT + RANSAC tform initialiser block.
    
    % Generate feature-based point clouds from RGB data and depth
    % correspondance using SURF feature detection + MSAC outlier removal
    [tForm2D,inlierIndices1,inlierIndices2] = findImageFeatures(colorFrameData,depthFrameData,i);
    
    % Verbose test for detecting mismatching counts in feature clouds.
    % inlierRemovedTest(inlierIndices1, inlierIndices2,depthFrameData,i);
    
    % Generate the point cloud consisting only of matched 3D features
    [featCloudFixed] = genFeatureCloud(inlierIndices1,depthFrameData,i-1);
    [featCloudMoving] = genFeatureCloud(inlierIndices2,depthFrameData,i);
    
    % Remove NaN values and reshape from NxMx3 to Nx3
    featCloudFixed = removeInvalidPoints(featCloudFixed);
    featCloudMoving = removeInvalidPoints(featCloudMoving);
    
    % Some zero values may remain for one of xyz dimensions. These are
    % effectively invalid points but are missed by the removeInvalidPoints
    % function. This function removes this points and finally ensures that
    % the number of points in each cloud are always identical.
%     [featCloudFixed,featCloudMoving] = pointCloudPairRemoveAllZeros(...
%         featCloudFixed,featCloudMoving);
    
    % Statement checks that args have been filtered appropriately and
    % contain same number of elements. Two options are presented here, the
    % first is a basic least squares rigid motion estimate via SVD. The
    % second uses a RANSAC-based opencv affine transform estimate, often
    % non-rigid...at present, neither improve upon the constant velocity
    % model with point-to-plane registration
    
    if featCloudMoving.Count == featCloudFixed.Count
        
        % SVD rigid transform estimation
        [R,t] = rigid_transform_3D(featCloudMoving.Location, featCloudFixed.Location);
        svdM = R;
        svdM(1,4) = t(1); svdM(2,4) = t(1); svdM(3,4) = t(1);
        svdM(4,:) = [0,0,0,1];
        
        % Convert to affine3d object
        
        svdM(isnan(svdM))=0;
        svdM = affine3d(svdM');
        tform = svdM; 
        
%         % OpenCV function call, built using mexopencv. Usually returns
%         % non-rigid transformation.
%         ransacM = cv.estimateAffine3D(featCloudMoving.Location, featCloudFixed.Location,...
%             'RansacThreshold',3,'Confidence',0.99);
%         ransacM(4,:) = [0 0 0 1];
%         
%         % Convert to affine3d object
%         
%         ransacM = affine3d(ransacM');
        
        
    end
    
    
    % Debugging code to visualise frame-by-frame alignments
    if (showPtCloudComparison)
        
        figure; %#ok<UNRCH>
        pcshowpair(pctransformNonRigid(ptCloudCurrent,tform),ptCloudScene,'VerticalAxis','Y','VerticalAxisDir','Down'...
            ,'MarkerSize',16)
        title(strcat('Difference between scene and frame',num2str(i)))
        xlabel('X(m)')
        ylabel('Y(m)')
        zlabel('Z(m)')
    end
    
    
    % Check for badly conditioned matrices
    if rcond(tform.T)<0.5
        tform = eye(4);
        tform = affine3d(tform); 
    end
    % Store input transformation from last frame
    initialTform = tform;
    
    % Apply ICP registration. 
    [tform,~,rmse] = pcregrigidModified(moving, fixed, 'Metric','pointToPlane',...
        'InlierRatio',0.7,'Extrapolate', true,'Verbose',false...
        ,'InitialTransform',tform);
 
    % Calculate absolute running velocity and acceleration between frames
    T1(:,1) = initialTform.T(4, 1:3)';
    T2(:,1) = tform.T(4, 1:3)';
    dT(i) = sqrt(sum((T1(:,1)-T2(:,1)).^2));
    dV(i) = sqrt((dT(i)-dT(i-1))).^2;
    
%     % Velocity filter. Re-estimate tform without initial
%     if (abs(dV(i)))>0.05
%         
%         [tform,~,rmse] = pcregrigidModified(moving, fixed, 'Metric','pointToPlane',...
%         'InlierRatio',0.7,'Extrapolate', true,'Verbose',false);
%     end
    
    
    % RMSE Filter
    if (rmse > 0.5)
        droppedFrameCount = droppedFrameCount+1;
        continue
    end
    
    % Store Euclidean dist of current transform
    L2dist(i)=rmse;
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    
    
    % Perform forward transformation
    
    ptCloudAligned = pctransformNonRigid(ptCloudCurrent, accumTform);
    
    % Transform camera position to current reference scene.
    camPlot = pctransformNonRigid(camCloud,accumTform);
    camTrack(i,1) = camPlot.Location(1,1);
    camTrack(i,2) = camPlot.Location(1,2);
    camTrack(i,3) = camPlot.Location(1,3);
    
    % Extract quaternion rotation data
    % Done use quaternion classdef by:
    % Mark Tincknell, MIT LL, 29 July 2011, revised 25 June 2015
    q = quaternion.rotationmatrix(tform.T(1:3,1:3));
    camTrack(i,4) = q.e(4,1);
    camTrack(i,5) = q.e(3,1);
    camTrack(i,6) = q.e(2,1);
    camTrack(i,7) = q.e(1,1);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
    
    % Update Camera Pose
    ptCloudScene = pcmerge(ptCloudScene, camPlot, mergeSize);
    
    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    drawnow('update')
    
end

% Release Hungry Variables
% clear colorFrameData depthFrameData


% Tracking and Kalman Filter Block
% Initialise for potentially 6 tracked targets
detectedLocation = zeros(6,3);

for i = find(isTracked, 1):finalFrame
    for j =1:6
        if trackingCoords(i,:,j);
            
            detectedLocation(j,:) = trackingCoords(i,:,j);
            
            % Configure filter using detected location
            kalmanFilter = configureKalmanFilter('ConstantAcceleration',...
                detectedLocation(j,:), [1 1 1]*1e5, [25, 10, 10], 25);
            
            % Initialise prediction
            predict(kalmanFilter);
            
            % Correct covariance
            projectedLocation(i,:,j) = correct(kalmanFilter, detectedLocation(1,:));
            
            % Now predict 10 steps ahead
            
            for k=i+1:1+10
               projectedLocation(k,:,j) = predict(kalmanFilter);  
            end
        end
    end
end

% Visualisation Block

% %transform the data parallel to viewing axes, specify angle
% angle = -6*pi/180;
% A = [1,0,0,0;...
%     0, cos(angle),-sin(angle), 0; ...
%     0, sin(angle), cos(angle), 0; ...
%     0 0 0 1];
% ptCloudScene = pctransform(ptCloudScene, affine3d(A));
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
    'Parent', hAxes)
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

% Visualise trajectory alone.
camCloud = pointCloud(camTrack(:,1:3));

% View Result
figure
pcshow(camCloud, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
    'MarkerSize',50)
title('Updated world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')


% Plot running RMSE and display number of dropped frames
L2dist = L2dist(L2dist~=0);
distFig = figure;
plot(L2dist); title(strcat('RMS Error Euclidean Distance between ICP frames. Avg('...
    ,num2str(mean(L2dist)),')'));
xlabel('Frame Number'); ylabel('RMSE Distance.');
disp(strcat('Done. ' ,num2str(droppedFrameCount),' frames dropped!'));
beep

% % Uncomment to perform benchmark evaluation
% % Save estimated trajectory file
% trajEstimate = horzcat(timestamps, camTrack(1:finalFrame,:));
% trajEstimate = trajEstimate';
% fileID = fopen('estimated_trajectory_RGBD_frei2_360_hemi.txt','w');
% fprintf(fileID,'%5.5f %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f %5.5f\n',trajEstimate);
% fclose(fileID);