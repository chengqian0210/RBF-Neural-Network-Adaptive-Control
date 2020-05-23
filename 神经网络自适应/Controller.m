function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
global node c b
node=5;
sizes = simsizes;
sizes.NumContStates  = node+1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [zeros(1,5),120];
c= [-1 -0.5 0 0.5 1;
    -1 -0.5 0 0.5 1];
b=2;
str = [];
ts  = [];
function sys=mdlDerivatives(t,x,u)
global node c b
yd=sin(t);
dyd=cos(t);
ddyd=-sin(t);

e=u(1);
de=u(2);
x1=yd-e;
x2=dyd-de;

kp=30;
kd=50;
K=[kp kd]';
E=[e de]'; 

Fai=[0 1;-kp -kd];
A=Fai';
Q=[500 0;0 500];
P=lyap(A,Q);

W=[x(1) x(2) x(3) x(4) x(5)]';
xi=[e;de];
h=zeros(5,1);
for j=1:1:5
    h(j)=exp(-norm(xi-c(:,j))^2/(2*b^2));
end
fxp=W'*h;

mp=x(node+1);

ut=1/mp*(-fxp+ddyd+K'*E);

B=[0;1];
gama=1200;
S=-gama*E'*P*B*h;
for i=1:1:node
    sys(i)=S(i);
end

eta=0.0001;
ml=100;
if (E'*P*B*ut>0)
    dm=(1/eta)*E'*P*B*ut;
end
if (E'*P*B*ut<=0)
    if (mp>ml)
    dm=(1/eta)*E'*P*B*ut;
    else
    dm=1/eta;
    end
end
sys(node+1)=dm;
 
function sys=mdlOutputs(t,x,u)
global node c b
yd=sin(t);
dyd=cos(t);
ddyd=-sin(t);

e=u(1);
de=u(2);
x1=yd-e;
x2=dyd-de;

kp=30;
kd=50;
K=[kp kd]';
E=[e de]';

W=[x(1) x(2) x(3) x(4) x(5)]';
xi=[e;de];
h=zeros(5,1);
for j=1:1:node
    h(j)=exp(-norm(xi-c(:,j))^2/(2*b^2));
end
fxp=W'*h;

mp=x(node+1);

ut=1/mp*(-fxp+ddyd+K'*E);

sys(1)=ut;
sys(2)=fxp;
sys(3)=mp;