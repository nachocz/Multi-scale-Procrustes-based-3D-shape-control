clf;
stepTicks=100;tamtext=12;vScale=10;
%% plot deformation
cubeAlphaF=0.1;
cubeAlphaE=0.3;
subplot(2,2,2);hold on;
visualize_map_on_target(S1, S2, T12);title('Current');axis equal;
for g=1:G
    patch('Faces',boxFaces , 'Vertices', gripperBox(:,:,g),'Facecolor', 'k','FaceAlpha',cubeAlphaF,'EdgeAlpha',cubeAlphaE);
end
view(45,45);
subplot(2,2,4);hold on;
visualize_map_on_source(S1, S2, T12);title('Target');
view(45,45);

%% plot error and amount of deformation
subplot(2,2,1);
errorVecsCellFormat{i}=diag(pdist2(V,Vt(map,:)));
meanError(i)=mean(diag(pdist2(V,Vt(map,:))));
tv=F;
v_edges_repe = [ tv(:,1:2); tv(:,2:3); tv(:,[3,1]) ];
v_edges = unique( sort(v_edges_repe,2) ,'rows');
na = size(v_edges,1);
p=Vinitial;
dInitial = sqrt( sum( ( p( v_edges(:,1),:)-p( v_edges(:,2),:) ).^2, 2) );
pCurrent=V;
dCurrent = sqrt( sum( ( pCurrent( v_edges(:,1),:)-pCurrent( v_edges(:,2),:) ).^2, 2) );
% v_stretch(:,itd+1)=dCurrent;
v_stretch(:,i) = (100 * dCurrent ./ dInitial )-100;
%%NEW PLOT WITH SHADED CURVE
yyaxis right;
meanStretch=mean(v_stretch);
y=meanStretch;
stdVstretch=std(v_stretch);
x = 1:numel(meanStretch);
x2 = [x, fliplr(x)];
curve1 = meanStretch + stdVstretch;
curve2 = meanStretch - stdVstretch;
inBetween = [curve1, fliplr(curve2)];
hold on
fill(x2, [y-stdVstretch  fliplr(y+stdVstretch)], [0 0 1], 'FaceAlpha',0.1)
hold on;
hold on;
plot(x, y, 'b', 'LineWidth', 1);
xt=round(xticks);
xticklabels(xt*dif_t);
set(gca,'FontSize',tamtext);
ylim([-80 80]);
title('')
xlabel('Time [s]','interpreter','latex')
ylabel('Object deformation (\%)','interpreter','latex')
ax = gca;
ax.YAxis(1).Color = 'r';
ax.YAxis(2).Color = 'b';

yyaxis left;
y = dPro;
x = 1:numel(y);
plot(x, y, 'r', 'LineWidth', 1);

xt=round(xticks);
xticklabels(xt*dif_t);
xlabel('$t$ [s]','interpreter','latex');
set(gca,'FontSize',tamtext);
title('')
xlabel('Time [s]','interpreter','latex')
ylabel('Boundary error [mm]','interpreter','latex')
% xlim([1 iMax]);
ylim([0 max(dPro(:))+0.1]);
xt=round(xticks);
xticklabels(xt*dif_t);
box on;

%% plot scale analysis
if i>1
    subplot(2,2,3); hold on;
    surf(weightedcommutProbabAccum,'EdgeAlpha',0)
    hold on; plot3(1:i,actionScale,ones(i,1).*10,'r',LineWidth=2);

    yt=yticks;
    yticklabels(string(round(scaleVec([1 yt(2:end)]),2)));
    ylabel('$r^*(t)$ [cm]','interpreter','latex')

    xt=round(xticks);
    xticklabels(xt*dif_t);
    xlabel('$t$ [s]','interpreter','latex');
end
