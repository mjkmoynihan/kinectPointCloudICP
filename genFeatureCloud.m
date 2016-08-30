function [featCloud] = genFeatureCloud(inlierIndices,depthFrameData,i)

Points = zeros(480,640);

for j = 1:length(inlierIndices)
    Points(inlierIndices(j,2),inlierIndices(j,1))=...
        depthFrameData(inlierIndices(j,2),inlierIndices(j,1),1,i);
    
end

featCloud = depthToCloud(Points);
end