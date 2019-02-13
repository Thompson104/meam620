function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls
persistent path

if size(varargin, 2) == 2
    map = varargin{1}; 
    path = varargin{2};
elseif ~isempty(varargin)
    path = varargin;
end


pos = zeros(1, 3);
vel = zeros(1, 3);
acc = zeros(1, 3);
yaw = 0;
yawdot = 0;

num_seg = size(path, 1) - 1;
s = path(1, :);
e = path(num_seg + 1, :);
tot_time = 30;

diff_vec = path(2:size(path, 1), :) - path(1:size(path, 1) - 1, :);
nor_vec = zeros(1, size(diff_vec, 1));
for i = 1:size(diff_vec, 1)
    nor_vec(i) = norm(diff_vec(i, :));
end
tot_len = sum(nor_vec);
cum_time = [-1e-10, cumsum(nor_vec)*tot_time/tot_len];
for i = 1:size(cum_time, 2) - 1
    s_i = path(i, :);
    e_i = path(i+1, :);
    if cum_time(i) < t && t <= cum_time(i + 1)
        delta_t = cum_time(i + 1) - cum_time(i);
        pos = s_i + (e_i - s_i)*(t - cum_time(i))/delta_t;
        vel = (e_i - s_i)/delta_t;
    end
end

if t > tot_time
    pos = e;
end

%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
