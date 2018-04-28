%% z dirction
clear;clc;close all;
x0 = [20,-5];
options = optimset('PlotFcns',@optimplotfval);
x=fminsearch(@opti_z,x0,options);
%% x dirction
clear;clc;close all;
x0 = [10,-5];
options = optimset('PlotFcns',@optimplotfval);
x=fminsearch(@opti_x,x0,options);
%% y dirction
clear;clc;close all;
x0 = [10,-5];
options = optimset('PlotFcns',@optimplotfval);
x=fminsearch(@opti_y,x0,options);