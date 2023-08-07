V=S1.surface.VERT(:,:);
Vt=S2.surface.VERT(:,:);
F=S1.surface.TRIV;
Ft=S2.surface.TRIV;
%% global cost estimation
[dPro(i),~,~] = procrustesCuiral(V(:,:),Vt(T12,:));
clear D hbinscontour estimatedV;
if i==1
    %% initial global procrustes, moving reference to minimal global error configuration
    [dPro,Z,transform] = procrustes(V(:,:),Vt(T12,:),'reflection',0,'scaling',0);
    Vt=Vt*transform.T+transform.c(1,:);
    b=1;
    %% Analyse target shape's geodesic distances
    edgt=edges(Ft);
    weightst=sum(((Vt(edgt(:,1),:)-Vt(edgt(:,2),:)).^2),2).^(1/2);
    EdgeTablet = table(edgt,'VariableNames',{'EndNodes'});
    Grapht=graph(EdgeTablet);
    Grapht.Edges.Weight = weightst;
    %"geodesic" distances in target shape
    dt = distances(Grapht);
    deuct=pdist2(Vt,Vt);
    dt(dt==inf)=deuct(dt==inf);
    %% compute closest points to grippers
    igripperPos(:,:)=gripperPos(:,:,i);
    gIndex = dsearchn(V,igripperPos);
    Dt(:,1:length(gIndex))=dt(:,T12(gIndex(:)));
    %% generate scale analysis vector
    scaleWindow=6;
    nScales=20;
    scaleVec=linspace(gridStep*2,(max(dt,[],'all')),nScales);
    nScales=length(scaleVec);
    %% global cost estimation
    [dPro(i),~,~] = procrustesCuiral(V(:,:),Vt(T12,:));
    clear D hbinscontour estimatedV;
    %% compute isolines (geodesic distances)
    d = distances(Graph);
    deuc=pdist2(V,V);
    d(d==inf)=deuc(d==inf);
    igripperPos(:,:)=gripperPos(:,:,i);
    gIndex = dsearchn(V,igripperPos);% gripper indexes in mesh...
    D(:,1:length(gIndex))=d(:,gIndex(:));
    %compute centroid paths and iso-distance bins/segments
    [~,hbinsAll]=computeisodistancesegments(V,F,scaleVec,gIndex(:),D);
    hbinscontour(:,:,:)=hbinsAll(:,:,:);
    %% multiscale procrustes analysis
    for scale=1:nScales
        for g=1:G
            clear ghbins;
            ghbins(:,:)=logical(hbinscontour(:,scale,g));
            [Eproc(scale,g,i),iTransform(:,:,scale,g),Z] = procrustesCuiral(V(ghbins,:),Vt(T12(ghbins),:));
            gNpoints(scale,i)=sum(ghbins);
            Eproc(scale,g,i)=Eproc(scale,g,i)*((gNpoints(scale,i))/size(hbinscontour,1));
            Edist(scale,g,i)=mean(vecnorm(Vt(T12(ghbins),:)-V(ghbins,:),2,2))*(sum(ghbins)/size(hbinscontour,1));
        end
    end

    for scale1=1:nScales %action generation considered scale
        for scale2=1:nScales %influence scale
            estimatedV(:,:,scale1,scale2)=V(:,:);
            for g=1:G
                clear ghbins estVincrement gTaction;
                ghbins(:,:)=logical(hbinscontour(:,scale2,g));
                gTaction(:,:)=[iTransform(:,:,scale1,g)];
                gTout = rigidtransformgeodesicpath(dif_t,eye(4),(gTaction));
                gnewV=((gTout)*[V ones(length(V),1)]')';
                gnewV(:,4)=[];
                estVincrement=gnewV(:,:)-V(:,:);
                estimatedV(:,:,scale1,scale2)=estimatedV(:,:,scale1,scale2)+estVincrement(:,:).*ghbins(:,:);
            end
            [estdPro(scale1,scale2,i),~,~] = procrustesCuiral(estimatedV(:,:,scale1,scale2),Vt(T12,:));
            estDotdPro(scale1,scale2,i)=estdPro(scale1,scale2,i)-dPro(i);
        end
    end
    iEstDotdPro(:,:)=estDotdPro(:,:,i);
    commutProbabAccum(:,i)=sum(iEstDotdPro,1)';
    clear minDotEst actionScale;
    [minDotEst(i),actionScale(i)]=min(commutProbabAccum(:,i));
    weightedcommutProbabAccum=zeros(nScales,1);
    weightedcommutProbabAccum(round(actionScale(i)),i)=-1;
else
    %% compute isolines for new iteration (geodesic distances)
    d = distances(Graph);
    deuc=pdist2(V,V);
    d(d==inf)=deuc(d==inf);
    igripperPos(:,:)=gripperPos(:,:,i);
    gIndex = dsearchn(V,igripperPos);% gripper indexes in mesh...
    D(:,1:length(gIndex))=d(:,gIndex(:));
    %compute centroid paths and iso-distance bins/segments
    [~,hbinsAll]=computeisodistancesegments(V,F,scaleVec,gIndex(:),D);
    hbinscontour(:,:,:)=hbinsAll(:,:,:);
    %% multiscale procrustes analysis
    b=1;
    for scale=max(round((actionScale(i-1)-scaleWindow)),1):min(round((actionScale(i-1)+scaleWindow)),nScales)
        for g=1:G
            clear ghbins;
            ghbins(:,:)=logical(hbinscontour(:,scale,g));
            [Eproc(scale,g,i),iTransform(:,:,scale,g),Z] = procrustesCuiral(V(ghbins,:),Vt(T12(ghbins),:));
            gNpoints(scale,i)=sum(ghbins);
            Eproc(scale,g,i)=Eproc(scale,g,i)*((gNpoints(scale,i))/size(hbinscontour,1));
            Edist(scale,g,i)=mean(vecnorm(Vt(T12(ghbins),:)-V(ghbins,:),2,2))*(sum(ghbins)/size(hbinscontour,1));
        end
    end

    for scale1=max(round((actionScale(i-1)-scaleWindow)),1):min(round((actionScale(i-1)+scaleWindow)),nScales) %action generation considered scale
        for scale2=max(round((actionScale(i-1)-scaleWindow)),1):min(round((actionScale(i-1)+scaleWindow)),nScales) %influence scale
            estimatedV(:,:,scale1,scale2)=V(:,:);
            for g=1:G
                clear ghbins estVincrement gTaction;
                ghbins(:,:)=logical(hbinscontour(:,scale2,g));
                gTaction(:,:)=[iTransform(:,:,scale1,g)];
                gTout = rigidtransformgeodesicpath(dif_t,eye(4),(gTaction));
                gnewV=((gTout)*[V ones(length(V),1)]')';
                gnewV(:,4)=[];
                estVincrement=gnewV(:,:)-V(:,:);
                estimatedV(:,:,scale1,scale2)=estimatedV(:,:,scale1,scale2)+estVincrement(:,:).*ghbins(:,:);
            end
            [estdPro(scale1,scale2,i),~,~] = procrustesCuiral(estimatedV(:,:,scale1,scale2),Vt(T12,:));
            estDotdPro(scale1,scale2,i)=estdPro(scale1,scale2,i)-dPro(i);
        end
    end
    iEstDotdPro(:,:)=estDotdPro(:,:,i);
    commutProbabAccum(:,i)=sum(iEstDotdPro,1)';
    %% actual error and error derivative validation
    iDeltaEproc(:,:)=Eproc(:,:,i)-Eproc(:,:,i-1);
    deltaEproc(:,:,i)=iDeltaEproc(:,:);
    iDeltaEdist(:,:)=Edist(:,:,i)-Edist(:,:,i-1);
    deltaEdist(:,:,i)=iDeltaEdist(:,:);
    dEstimationQuality(:,i)=ones(nScales,1).*10;
    for scale2=max(round((actionScale(i-1)-scaleWindow)),1):min(min(round((actionScale(i-1)+scaleWindow)),nScales),size(antEstimatedV,4))
        ghbins(:,:)=logical(hbinscontour(:,scale2,g));
        [dEstimationQuality(scale2,i)]=procrustesCuiral(antEstimatedV(:,:,round(actionScale(i-1)),scale2),V(:,:))./length(V);
    end
    normalisedEstimationQuality(:,i)=1./exp(BETA_PARAM*dEstimationQuality(:,i));
    weightedEstimation(:,:,i)=iEstDotdPro.*normalisedEstimationQuality(:,i);
    weightedcommutProbabAccum(:,i)=sum(weightedEstimation(:,:,i),1)';
    %q(t,r) in the paper
    weightedcommutProbabAccum(:,i)=weightedcommutProbabAccum(:,i)./max(abs(weightedcommutProbabAccum(:,i)));
    %% gradient computation and r^* update
    FX = gradient(weightedcommutProbabAccum(:,i));
    optScale(1)=actionScale(i-1);
    kkMax=10;
    for kk=2:kkMax
        FXantactscale=interp1(1:nScales,FX,optScale(kk-1));
        optScale(kk)=optScale(kk-1)-FXantactscale;
    end
    actionScale(i)=optScale(end);
end
antEstimatedV=estimatedV;
%% gripper actions
for g=1:G
    pg=igripperPos(g,:);
    ghbins(:,:)=logical(hbinscontour(:,round(actionScale(i)),g));
    [dpg,gTaction,~]=procrustesCuiral(V(ghbins,:)-pg,Vt(T12(ghbins),:)-pg);
    gTout = rigidtransformgeodesicpath(abs(weightedcommutProbabAccum(round(actionScale(i)),i)).*dif_t,eye(4),(gTaction));
    iTaction(:,:,g,i)=(gTout);
    rotvec(:,i,g)=rotationMatrixToVector(gTout(1:3,1:3));
    tvec(:,i,g)=gTout(1:3,4)';
end
