function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

R=5;
z_final=2.5;
time2goal=11;
dthetadt=2*pi/time2goal;

if t>time2goal
%     theta=0;
    x_des=R*cos(0);
    y_des=R*sin(0);
    z_des=z_final;
    x_vel=0;
    y_vel=0;
    z_vel=0;
    x_acc=0;
    y_acc=0;
    z_acc=0;
else
    theta=2*pi*t/time2goal;
    x_des=R*cos(theta);
    y_des=R*sin(theta);
    z_des=z_final*t/time2goal;
    x_vel=-R*dthetadt*sin(theta);
    y_vel=R*dthetadt*cos(theta);
    z_vel=z_final/time2goal; 
    x_acc=-R*dthetadt^2*cos(theta);
    y_acc=-R*dthetadt^2*sin(theta);
    z_acc=0;
end

pos = [x_des; y_des; z_des];
vel = [x_vel;y_vel; z_vel];
acc = [x_acc;y_acc ; z_acc];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
