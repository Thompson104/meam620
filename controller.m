function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
%
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%
% qn: quadrotor number, should always be 1
%
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================

% Geometric Controller
% Gains

K_R = 10000*diag([2,2,1]); % Rotational error gain
K_omega = 800*diag([2,2,1]); % Angular gain

wn_pos = 5*[1;1;2.3];
zeta_pos = 0.8*[1;1;1;];
K_p = diag(wn_pos.^2);
K_d = diag(2.*zeta_pos.*wn_pos);

% Calculate u_1
% Note the inconsistency, the qn.acc_des is target accel, but we are being
% consistent so acc_des is acc_des, and qn.acc_des is target acc

acc_d = qd{qn}.acc_des - K_d*(qd{qn}.vel - qd{qn}.vel_des) - K_p*(qd{qn}.pos - qd{qn}.pos_des);
force_des = params.mass .* acc_d + [0 0 params.mass*params.grav]';
rot_A2B = eulzxy2rotmat(qd{qn}.euler); % R
b3 = rot_A2B * [0; 0; 1;];
u_1 = b3' * force_des;

% Calculate u_2

b3_des = force_des / max(norm(force_des), 1e-5);

psi_d = qd{qn}.yaw_des;

a_phi = [cos(psi_d); sin(psi_d); 0];
cross_b3des_aphi = cross(b3_des, a_phi);
b2_des = cross_b3des_aphi / max(norm(cross_b3des_aphi), 1e-5);

rot_des = [cross(b2_des,b3_des), b2_des, b3_des]; % R_des

error_R_mat = 0.5*(rot_des'*rot_A2B - rot_A2B' * rot_des);
error_R = [error_R_mat(3,2); error_R_mat(1,3); error_R_mat(2,1);];
omega_des = [0,0,0]'; % No error, since desired omega is simply 0.
error_omega = qd{qn}.omega - omega_des;

u_2 = params.I*(-K_R*error_R - K_omega*error_omega);
% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

% TODO
phi_des   = 0;
theta_des = 0;
psi_des   = 0;

%
%
%
u = [u_1; u_2]; % control input u


% Thrust
F    = u(1);       % This should be F = u(1) from the project handout
% Moment
M    = u(2:4);     % note: params.I has the moment of inertia


% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);

    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
