function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

if t <= 10
    pos = [5*cos(0.2*pi*t); 5*sin(0.2*pi*t); 0.25*t];
    vel = [-pi*sin(0.2*pi*t); pi*cos(0.2*pi*t); 0.25];
    acc = [-0.2*pi^2*cos(0.2*pi*t); -0.2*pi^2*sin(0.2*pi*t); 0];
    yaw = 0.2*pi*t;
    yawdot = 0.2*pi;
elseif t > 10
    pos = [5; 0; 2.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
