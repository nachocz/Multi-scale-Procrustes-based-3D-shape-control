function [centroids,histogramBinsAll] = computeisodistancesegments(V,F,scaleVec,gIndex,D)
histogramBinsAll=[];
centroids=[];
rBins=[];
scaleRes=scaleVec(2)-scaleVec(1);
rBins=scaleVec;
nPoints=size(D,1);
nBins=length(rBins);
belowValueMatrix=repmat(rBins(1:(end-1)),[nPoints 1]);
overValueMatrix=repmat(rBins(1:end),[nPoints 1]);
for ngripper=1:length(gIndex)
    distancesRepeatedMatrix=repmat(D(:,ngripper),[1 nBins]);
    histogramBins=distancesRepeatedMatrix.*(distancesRepeatedMatrix<=overValueMatrix);%.*(distancesRepeatedMatrix>=belowValueMatrix);
    histogramBins(histogramBins>0)=1;
    histogramBins(gIndex(ngripper),:)=1;
    Pxrep=repmat(V(:,1),[1 nBins]);
    Pyrep=repmat(V(:,2),[1 nBins]);
    Pzrep=repmat(V(:,3),[1 nBins]);
    centroidsX=sum((Pxrep.*logical(histogramBins)),1)./sum(histogramBins,1);
    centroidsY=sum((Pyrep.*logical(histogramBins)),1)./sum(histogramBins,1);
    centroidsZ=sum((Pzrep.*logical(histogramBins)),1)./sum(histogramBins,1);
    centroids(:,1:3,ngripper)=[V(gIndex(ngripper),:); centroidsX' centroidsY' centroidsZ'];
    histogramBinsAll(:,:,ngripper)=histogramBins(:,:);
end
end

