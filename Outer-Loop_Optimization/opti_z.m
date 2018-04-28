function y=opti_z(x)
assignin('base','Kp_z',x(1));
assignin('base','Kd_z',x(2));
% assignin('base','Kp_z',25.2914);
% assignin('base','Kd_z',-6.4704);

assignin('base','mQ',0.55);
[tt,xx,yy]=sim('Sim_Plant_z',[0,5]);
y=yy(end,1);
if max(yy(:,2))>0.2*(1.03), y = 5.5*y; end
end

