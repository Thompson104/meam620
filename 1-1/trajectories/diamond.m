function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

if t <= 2.5
    pos = [t/10; 0.4*sqrt(2)*t; 0.4*sqrt(2)*t];
    vel = [1/10; 0.4*sqrt(2); 0.4*sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif (2.5 < t) && (t <= 5)
    pos = [t/10; -0.4*sqrt(2)*t + 2*sqrt(2); 0.4*sqrt(2)*t];
    vel = [1/10; -0.4*sqrt(2); 0.4*sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif (5 < t) && (t <= 7.5)
    pos = [t/10; -0.4*sqrt(2)*t + 2*sqrt(2); -0.4*sqrt(2)*t + 4*sqrt(2)];
    vel = [1/10; -0.4*sqrt(2); -0.4*sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif (7.5 < t) && (t <= 10)
    pos = [t/10; 0.4*sqrt(2)*t - 4*sqrt(2); -0.4*sqrt(2)*t + 4*sqrt(2)];
    vel = [1/10; 0.4*sqrt(2); -0.4*sqrt(2)];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
elseif t > 10
    pos = [1; 0; 0];
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
