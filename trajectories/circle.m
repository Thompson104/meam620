function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

t_f = 12.7; % Final time

    function pos_des = get_pos_des(t_d, t_final)
        radius = 5;
        z_max = 2.5;
        k = max(min(t_d/t_final,1),0); % length along path normalized
            
        theta = 2*pi*k;
        x = radius*cos(theta);
        y = radius*sin(theta);
        z = z_max*k;

        pos_des = [x; y; z];
    end

% Velocity is just finite differences in the trajectory 
pos = get_pos_des(t, t_f);
timestep = 0.02; % to calculate velocity
pos_prev = get_pos_des(t-timestep, t_f);
vel = (pos-pos_prev) / timestep;

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
