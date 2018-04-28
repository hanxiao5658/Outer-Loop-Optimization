function y=opti_y(x)
assignin('base','Kp_y',x(1));
assignin('base','Kd_y',x(2));
assignin('base','Kp_z',25.2914);
assignin('base','Kd_z',-6.4704);

assignin('base','mQ',0.55);
[tt,xx,yy]=sim('Sim_Plant_y',[0,5]);
y=yy(end,1);
if max(yy(:,2))>0.2*(1.03), y = 5.5*y; end
end

