function [sys,x0,str,ts] = LG_ID_ss_Model(t,x,u,flag) 
%%
 switch flag
    case 0
    [sys,x0,str,ts]=mdlInitializeSizes(); 
    case 1
    sys=mdlDerivatives(t,x,u); 
    case 3
    sys=mdlOutputs(t,x,u); 
    case { 2, 4, 9 }
    sys = []; 
    otherwise 
    error(['Unhandled flag = ',num2str(flag)]); 
end 
function [sys,x0,str,ts]=mdlInitializeSizes()
%% 
sizes = simsizes; 
sizes.NumContStates = 12;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 9;
sizes.NumInputs = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = zeros(12,1);
str = [];
ts = [0 0];
function sys=mdlDerivatives(t,x,u)
%% Model states and controllors:
x1 = x(1);
phi = x(2);
x3 = x(3);
theta = x(4);
x5 = x(5);
dFt =  x(6);
dxQ = x(7);
dyQ = x(8);
dzQ = x(9);
xQ = x(10);
yQ = x(11);
zQ = x(12);

phi_des = u(1);
theta_des = u(2);
dFt_des = u(3);

g = 9.8;
mQ = 0.5645;

x1dot = -41.33*phi + 22.59*phi_des;
x2dot = x1 - 7.14*phi + 2.5*phi_des;
x3dot = -52.61*theta + 29.93*theta_des;
x4dot = x3 - 7.67*theta + 1.84*theta_des;
x5dot = -222.77*dFt + 299.63*dFt_des;
x6dot = x5 - 16.36*dFt + 10.41*dFt_des;
temp = mQ\(euler2rotMat(phi,theta,0)*[0,0,dFt+mQ*g]'-mQ*[0,0,g]');
x7dot = temp(1);
x8dot = temp(2);
x9dot = temp(3);
x10dot = dxQ;
x11dot = dyQ;
x12dot = dzQ;

sys = [x1dot;x2dot;x3dot;x4dot;x5dot;x6dot;...
    x7dot;x8dot;x9dot;x10dot;x11dot;x12dot]; 
function sys = mdlOutputs(t,x,u)
%% Model states and controllors:
x1 = x(1);
phi = x(2);
x3 = x(3);
theta = x(4);
x5 = x(5);
dFt =  x(6);
dxQ = x(7);
dyQ = x(8);
dzQ = x(9);
xQ = x(10);
yQ = x(11);
zQ = x(12);

phi_des = u(1);
theta_des = u(2);
dFt_des = u(3);

%% TransForm Matrix
sys = [phi;theta;dFt;xQ;dxQ;yQ;dyQ;zQ;dzQ]; 

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];