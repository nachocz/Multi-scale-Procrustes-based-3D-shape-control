% Demo (simple/vanilla implementation) of "Multi-scale Procrustes-based 3D shape
% control" by I.Cuiral-Zueco et. al. (2023)
%% Workspace setup
clc; close all; clear all;
addpath(genpath('.'));
%% Parameters
dif_t=0.1; %time step in [s]
%inner meshing params
resolutionRate=0.2;
maxvol=50;
% gripper size [cm]
gripperSize=1;
% control param
BETA_PARAM=10000;
%% shape files dirs
experimentName='experiment_1';
s2_name = './cuiral-meshes/shape_target.stl';
s1_name = './cuiral-meshes/shape.stl';
load('surfacemap.mat');
%% read the mesh and initialise grippers
S1 = MESH.MESH_IO.read_shape([s1_name]);
S2 = MESH.MESH_IO.read_shape([s2_name]);
F=S1.surface.TRIV;V=S1.surface.VERT;
Ft=S2.surface.TRIV;Vt=S2.surface.VERT;
%mesh the inside of the object (for simulation purposes)
%this function is part of iso2mesh toolbox (http://iso2mesh.sf.net)
[node,elem,face]=surf2mesh(V,F,min(V),max(V),resolutionRate,maxvol);
%extracting/isolating surface data...
F=face(:,1:3);F=sortrows(F);
Vid=unique(F(:));V=node(Vid,:);
[~,F]=ismember(F,Vid);
Vinitial=V; S1.VERT=V;

%%perform mesh generation (target shape)
[nodet,elemt,facet]=surf2mesh(Vt,Ft,min(Vt),max(Vt),resolutionRate,25);
%extracting/isolating surface data...
Ft=facet(:,1:3);Ft=sortrows(Ft);
Vidt=unique(Ft(:));Vt=nodet(Vidt,:);
[~,Ft]=ismember(Ft,Vidt);

%update shape structs
S1.surface.VERT=V; S1.surface.TRIV=F;
S1.surface.X=V(:,1);S1.surface.Y=V(:,2);S1.surface.Z=V(:,3);

S2.surface.VERT=Vt; S2.surface.TRIV=Ft;
S2.surface.X=Vt(:,1);S2.surface.Y=Vt(:,2);S2.surface.Z=Vt(:,3);

%check mesh approx resolution
v_edges = unique( sort([ F(:,1:2); F(:,2:3); F(:,[3,1]) ],2) ,'rows');
na = size(v_edges,1);
dInitial = sqrt( sum( ( Vinitial( v_edges(:,1),:)-Vinitial( v_edges(:,2),:) ).^2, 2) );
gridStep=mean(dInitial);
%% creacion grippers grandes
%Gripper indexes
G=length(gIndex);
% crear grafo de puntos del objeto y obtener indices de gripper grandes
gripperPos=V(gIndex,:);
edg=edges(F);
weights=sum(((V(edg(:,1),:)-V(edg(:,2),:)).^2),2).^(1/2);
EdgeTable = table(edg,'VariableNames',{'EndNodes'});
Graph=graph(EdgeTable);
Graph.Edges.Weight = weights;
d = distances(Graph);
D(:,1:length(gIndex))=d(:,gIndex(:));
%obtener indices grippers grandes, solo primera iteración... no cambian
for g=1:G
    largeGripIndex{g} = [nearest(Graph,gIndex(g),gripperSize); gIndex(g)];
end

%% initialise gripper boxes, for visualisation
boxSize=gripperSize;
boxCoord=[0 0 0; 0 0 1; 1 0 1; 1 0 0; 0 1 0; 0 1 1; 1 1 1; 1 1 0; ]-0.5;
boxFaces = [1 4 3 2; 5 8 7 6; 1 2 6 5; 3 4 8 7; 2 3 7 6; 1 5 8 4];
for g=1:G
    gripperBox(:,:,g)=V(gIndex(g),:)+boxCoord.*boxSize;
end

%% Global procrustes rigid adjustment of the reference shape
[dp,Tg,X2] = procrustesCuiral(Vt(T12,:),V);
Vt=(Tg*[Vt ones(length(Vt),1)]')';
Vt=Vt(:,1:3);
S2.surface.VERT=Vt; S2.surface.TRIV=Ft;
S2.surface.X=Vt(:,1);S2.surface.Y=Vt(:,2);S2.surface.Z=Vt(:,3);

%% Surface map visualisation
cubeAlphaF=0.1;cubeAlphaE=0.4;
figure;subplot(1,2,1);
visualize_map_on_target(S1, S2, T12);title('Source');axis equal;
hold on;
for g=1:G
    patch('Faces',boxFaces , 'Vertices', gripperBox(:,:,g),'Facecolor', 'k','FaceAlpha',cubeAlphaF,'EdgeAlpha',cubeAlphaE);
end
subplot(1,2,2);
visualize_map_on_source(S1, S2, T12);title('Target');

%% main loop
loopFigure=figure('Renderer', 'painters', 'Position', [10 10 1280 720]);
iMax=500;map=[];
%video saving
writerObj = VideoWriter('demo-video.avi');
writerObj.FrameRate = round(1/dif_t);
open(writerObj);

for i=1:iMax
    gripperPos(:,:,i)=V(gIndex(:),:);
    %% shape control strategy
    tic
    multiscaleprocrustescontrol;
    timecost(i)=toc;
    iHz=1/timecost(i)
    %% update gripper configuration according to control actions
    extendedGripperIndex=[];Uaction=[];
    for g=1:G
        scaleIndex(i,g)=actionScale(i);
        gIndexes=cell2mat(largeGripIndex(g));
        gTout(:,:)=iTaction(:,:,g,i);
        newV=((gTout)*[V ones(length(V),1)]')';
        newGripperBox=((gTout)*[gripperBox(:,:,g) ones(length(gripperBox(:,:,g)),1)]')';
        gripperBox(:,:,g)=newGripperBox(:,1:3);
        iTaction(:,:,g,i)=(gTout);
        newV(:,4)=[];
        gUaction=newV(gIndexes,:);
        Uaction=[Uaction; gUaction];
        extendedGripperIndex=[extendedGripperIndex;gIndexes];
    end

    %% simulate deformation
    Vant=V;
    grippersTargPos=Uaction;
    [node]=arap(node,elem(:,1:4),Vid(extendedGripperIndex), grippersTargPos);%,'Energy','elements');
    V=node(Vid,:);
    %%Update shape structs
    S1.surface.TRIV=F;S1.surface.VERT=V;S1.surface.X=V(:,1);S1.surface.Y=V(:,2);S1.surface.Z=V(:,3);
    S2.surface.TRIV=Ft;S2.surface.VERT=Vt;S2.surface.X=Vt(:,1);S2.surface.Y=Vt(:,2);S2.surface.Z=Vt(:,3);
    N=length(V);
    errorVec=Vt(T12,:)-V(:,:);
    errorVecReshape=reshape(errorVec',[3*N 1]);

    %% Visualisation
    plotLoopFigure;
    writeVideo(writerObj, getframe(gcf));
end
%%
close(writerObj);


