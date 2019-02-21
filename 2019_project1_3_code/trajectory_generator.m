function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
persistent traj2follow pos_prev
if nargin>2
    indWaypoints=findChangeNonLin(path);
    waypoints=path(indWaypoints,:,:);
    waypoints=[path(1,:,:);waypoints;path(end,:,:)];
    target_speed = 0.3; % Meters per second
    traj2follow = generate_trajectory(waypoints,target_speed);
    pos_prev=path(1,:,:);
    desired_state = [];
else
    desired_state = eval_trajectory(traj2follow, t,pos_prev);
    pos_prev=desired_state.pos';
end


%% Trajectory Generation function
% Returns a function that takes time t as an input and returns a position
% on the trajectory.
function traj2follow_ = generate_trajectory(waypoints_,target_speed_)
  segment_vectors = waypoints_(2:end,:) - waypoints_(1:end-1,:);
  % the length of each segment
  % this is row-wise norm 
  segment_lengths = sqrt(diag(segment_vectors*segment_vectors'));
  total_path_length = sum(segment_lengths);
  cumu_segment_lengths = [0; cumsum(segment_lengths)];
  t_f = total_path_length/target_speed_;
  traj2follow_ = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length);
end

%% Evaluate trajectory given trajectory function handle and time t
% Assumes that time starts at 0, and finishes at a desiresd time.
function desired_state_ = eval_trajectory(traj2follow_, t_,pos_prev)
  % Velocity is just finite differences in the trajectory

  pos = traj2follow_(t_);
  timestep = 0.02; % to calculate velocity
%   pos_prev = trajectory_generator(t_-timestep);
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