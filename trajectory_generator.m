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

% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state = [];
persistent trajectory_generator;
target_speed = 0.3; % Meters per second
if isempty(varargin)
  desired_state = eval_trajectory(trajectory_generator, t);
else
  % we need to generate a trajectory
  map = varargin{1};
  path = varargin{2};
  trajectory_generator = generate_trajectory(map, path);
end

%% Trajectory Generation function
% Returns a function that takes time t as an input and returns a position
% on the trajectory.
function trajectory_generator_ = generate_trajectory(map_, waypoints_)
  segment_vectors = waypoints_(2:end,:) - waypoints_(1:end-1,:);
  % the length of each segment
  % this is row-wise norm 
  segment_lengths = sqrt(diag(segment_vectors*segment_vectors'));
  total_path_length = sum(segment_lengths);
  cumu_segment_lengths = [0; cumsum(segment_lengths)];
  t_f = total_path_length/target_speed;
  trajectory_generator_ = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length);
end

%% Evaluate trajectory given trajectory function handle and time t
% Assumes that time starts at 0, and finishes at a desiresd time.
function desired_state_ = eval_trajectory(trajectory_generator_, t_)
  % Velocity is just finite differences in the trajectory

  pos = trajectory_generator_(t_);
  timestep = 0.02; % to calculate velocity
  pos_prev = trajectory_generator(t_-timestep);
  vel = (pos-pos_prev) / timestep;

  acc = [0; 0; 0];
  yaw = 0;
  yawdot = 0;

  desired_state_.pos = pos(:);
  desired_state_.vel = vel(:);
  desired_state_.acc = acc(:);
  desired_state_.yaw = yaw;
  desired_state_.yawdot = yawdot;
end

end
