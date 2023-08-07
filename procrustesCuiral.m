function [dp,Tg,X2] = procrustesCuiral(X,Y)
%ICZ unizar 2022
%transform from X to Y

if size(X,1)>size(X,2)
    X=X';Y=Y';
    trasp=1;
else
    trasp=0;
end
meanX=mean(X')';
meanY=mean(Y')';

X=[X; ones(1,length(X))];
Y=[Y; ones(1,length(Y))];

[Ug,Sg,Vg]=svd((Y-[meanY; 1])*(X-[meanX; 1])');
iSg=diag(Sg);
M=eye(3);M(3,3)=det(Ug)*det(Vg);
Rg=Ug(1:3,1:3)*M*Vg(1:3,1:3)';
tg=mean(Y(1:3,:),2)-Rg*mean(X(1:3,:),2);
Tg=[Rg tg; 0 0 0 1];

X2=Tg*X;
X2=X2(1:3,:);

% dp=norm(X2(1:3,:),'fro')^2+norm(Y,'fro')^2-2*trace(X2(1:3,:)'*Y(1:3,:));
b=1;
dp=norm(vecnorm(X2-Y(1:3,:)));

if trasp
    X2=X2';
end


end

