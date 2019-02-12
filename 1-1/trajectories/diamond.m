function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
t_f = 10; % Final time

    function pos_des = get_pos_des(t_d, t_final)
        k = max(min(t_d/t_final,1),0); % length along path normalized
        node_1 = [0;   0;       0;        ];
        node_2 = [1/4; sqrt(2); sqrt(2);  ];
        node_3 = [0.5; 0;       2*sqrt(2);];
        node_4 = [3/4; -sqrt(2); sqrt(2)  ];
        node_5 = [1;   0;        0        ];

        nodes = [node_1, node_2, node_3, node_4, node_5];

        ls = [norm(node_1 - node_2), norm(node_2 - node_3), norm(node_3 - node_4), norm(node_4 - node_5)];
        total_length = sum(ls);

        % TODO: redo, this is kinda ugly 
        k_length = k*total_length;

        pos_des = node_5;

        if k_length < ls(1)
            alpha = k_length/ls(1); % fraction along subpath
            pos_des = alpha.*node_2 + (1-alpha).*node_1;
        elseif k_length < ls(1)+ls(2)
            alpha = (k_length - ls(1))/ls(2); % fraction along subpath
            pos_des = alpha.*node_3 + (1-alpha).*node_2;
        elseif k_length < ls(1)+ls(2)+ls(3)
            alpha = (k_length - ls(1) - ls(2))/ls(3); % fraction along subpath
            pos_des = alpha.*node_4 + (1-alpha).*node_3;
        elseif k_length < ls(1)+ls(2)+ls(3)+ls(4)
            alpha = (k_length - ls(1) - ls(2) - ls(3))/ls(4); % fraction along subpath
            pos_des = alpha.*node_5 + (1-alpha).*node_4;
        end


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
