function Tout = rigidtransformgeodesicpath(t,H0,H1)
%ICZ
%Extracts geodesic transformation between two transformations for a given
%time parameter t in [0,1]
H=H1*[H0(1:3,1:3)' -(H0(1:3,1:3)')*H0(1:3,4);0 0 0 1];
S=logm(H(1:3,1:3));
s0=S(3,2);s1=S(1,3);s2=S(2,1);
theta=sqrt(s0*s0+s1*s1+s2*s2);
thetaSqr=theta*theta;
%inv V1
if theta>0.00001 %tol to 0...
    c=(1-(theta*sin(theta))/(2*(1-cos(theta))))/thetaSqr;
    invV1=eye(3)-0.5*S+c*S*S;
else
    invV1= eye(3);
end
U=invV1*H(1:3,4);
angle=t*theta;
if theta>0.00001 %tol to 0...
    interpR=eye(3)+(sin(angle)/theta)*S+((1-cos(angle))/thetaSqr)*S*S;
else
    interpR=eye(3);
end
if theta>0.00001 %tol to 0...
    thetaCub=theta*thetaSqr;
    c0=(1-cos(angle))/thetaSqr;
    c1=(angle-sin(angle))/thetaCub;
    interpTTimesV=t*eye(3)+c0*S+c1*S*S;
else
    interpTTimesV=t*eye(3);
end
interpH(1:3,1:3)=interpR*H0(1:3,1:3);
interpH(1:3,4)=interpR*H0(1:3,4)+interpTTimesV*U;
if sum(sum(isnan(interpH)))>0
    b=1;
end
Tout=[interpH;0 0 0 1];
end

