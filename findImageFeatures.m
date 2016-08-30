function [tForm2D,inlierIndices1,inlierIndices2,j] = findImageFeatures(colorFrameData,depthFrameData,i) 

%Read the stereo images.
I1 = rgb2gray(colorFrameData(:,:,:,i-1));
I2 = rgb2gray(colorFrameData(:,:,:,i));

% figure; 
% subplot(2,1,1); imshow(I1); 
% subplot(2,1,2); imshow(I2); 

%Find the features.
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

%Extract the neighborhood features.
[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);

%Match the features. K nearest neighbour
indexPairs = matchFeatures(features1,features2,'Unique',true);

%Retrieve the locations of the corresponding points for each image.
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

%Visualize the corresponding points. You can see the effect of translation 
%between the two images despite several erroneous matches.
% figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);

% Remove outliers via MSAC (RANSAC Variant) 
[tForm2D,inlierPts2,inlierPts1] = estimateGeometricTransform(matchedPoints2,matchedPoints1,'affine');
inlierIndices1 = round(inlierPts1.Location); 
inlierIndices2 = round(inlierPts2.Location); 

% Remove points that correspond to invalid depth value in order to maintain
% equally-sized point clouds
j = 1;
flag = true;
count = length(inlierIndices1);

while flag
 
   if depthFrameData(inlierIndices1(j,2),inlierIndices1(j,1),1,i-1) == 0 ...
           ||  depthFrameData(inlierIndices2(j,2),inlierIndices2(j,1),1,i) == 0
  
      inlierIndices1(j,:) = [];
      inlierIndices2(j,:) = [];
      count = count-1;
      
   end
   
     
   if j < count
       j  = j+1;
   else
      flag = false;
   end

   
end




% figure; showMatchedFeatures(I1,I2,inlierPts2,inlierPts1);
% title('Matched inlier points');

% outputView = imref2d(size(I1));
% Ir = imwarp(I2,tform,'OutputView',outputView);
% % figure; imshow(Ir);
% % title('Recovered image');
% 
% mask = im2bw(Ir,0.003); 
% % figure; imagesc(mask); title('Binary Mask');

end 

