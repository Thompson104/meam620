function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

t_f = 11.4; % Final time

k = max(min(t/t_f,1),0); % length along path normalized

radius = 5;
z_max = 2.5;

theta = 2*pi*k;
x = radius*cos(theta);
y = radius*sin(theta);
z = z_max*k;

pos = [x; y; z];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
