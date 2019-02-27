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


desired_state = [];
persistent traj;
target_speed = 0.8; % Meters per second
dist_gain = 3.8;
too_close_to_obs = 0.2;
use_mat_splines = true;

if isempty(t)
  % we need to generate a trajectory
  traj = generate_trajectory(map, path);
else
    if use_mat_splines
        desired_state = eval_trajectory(traj, t);
    else
        desired_state = eval_traj_smooth(traj, t);
    end
end


%% Trajectory Generation function
% Returns a function that takes time t as an input and returns a position
% on the trajectory.
function trajectory_generator_ = generate_trajectory(map_, untrimmed_waypoints_)
  %waypoints_ = trim_path(map_, untrimmed_waypoints_, dist_gain);
%  flipped_untrimmed_waypoints_ = flipud(untrimmed_waypoints_)
%  flipped_trim1_waypoints_ = trim_path_raytrace(map_, flipped_untrimmed_waypoints_, too_close_to_obs, dist_gain);
%
%  % now we trim it the other way
%  trim1_waypoints_ = flipud(flipped_trim1_waypoints_);
%  sparse_waypoints_ = trim_path_raytrace(map_, trim1_waypoints_, too_close_to_obs, dist_gain);

  flipped_untrimmed_waypoints_ = flipud(untrimmed_waypoints_);
  flipped_keep1_waypoints_ = keep_path_raytrace(map_, flipped_untrimmed_waypoints_, too_close_to_obs, dist_gain);

  % now we trim it the other way
  keep1_waypoints_ = flipud(flipped_keep1_waypoints_);
  keep2_waypoints_ = keep_path_raytrace(map_, untrimmed_waypoints_, too_close_to_obs, dist_gain);

  keep_waypoints = or(keep1_waypoints_, keep2_waypoints_);
  % trim one more time
  to_trim_waypoints = untrimmed_waypoints_(keep_waypoints,:,:);
  sparse_waypoints_ = trim_path_raytrace(map_, to_trim_waypoints, too_close_to_obs+0.05, dist_gain);

  waypoints_ = add_intermediate_points(sparse_waypoints_);

  [cumu_segment_lengths, t_f, total_path_length] = process_waypoints(waypoints_);
  
  trajectory_generator_straight = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length);
  trajectory_generator_spline = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length, 'spline');

  % test
  test_t = 0:0.1:t_f;
  straight_traj = trajectory_generator_straight(test_t);
  spline_traj = trajectory_generator_spline(test_t);
  spline_length = sum(vecnorm(diff(spline_traj),2,2));
  fprintf('Total path length %f, total spline length %f \n', [total_path_length, spline_length]);
  trajectory_generator_ = trajectory_generator_spline;
  if (spline_length - total_path_length) / total_path_length > 0.2
      % add more waypoints, path length too long
      disp('spline not dense enough')
      dense_waypoints_ = add_intermediate_points(waypoints_);
      [cumu_segment_lengths, t_f, total_path_length] = process_waypoints(dense_waypoints_);
      trajectory_generator_dense = @(t) interp1(cumu_segment_lengths, dense_waypoints_, max(min(t,t_f),0)/t_f*total_path_length, 'spline');
      dense_spline_traj = trajectory_generator_dense(test_t);
      dense_spline_length = sum(vecnorm(diff(dense_spline_traj),2,2));
      if (dense_spline_length - total_path_length) / total_path_length > 0.2
          % path length still too long, just go with straight
          trajectory_generator_ = trajectory_generator_straight;
      else
          % path length good
          trajectory_generator_ = trajectory_generator_dense;
      end
  end

  % these can't be too far away, if they are they we have to add more waypoints and repeat

  % figure(6)
  % plot_path(map, trajectory_generator_(test_t))
  if ~use_mat_splines % using splines
      % use the min snap stuff
      %evenly sample waypoints
      trajectory_generator_ = smooth_wp(waypoints_);

  end


end

function [cumu_seg_len, t_fin, total_path_length] = process_waypoints(wp)
    segment_vectors = wp(2:end,:) - wp(1:end-1,:);
    % the length of each segment
    % this is row-wise norm
    segment_lengths = sqrt(diag(segment_vectors*segment_vectors'));
    total_path_length = sum(segment_lengths);
    cumu_seg_len = [0; cumsum(segment_lengths)];
    t_fin = total_path_length/target_speed;
end

%% Evaluate trajectory given trajectory function handle and time t
% Assumes that time starts at 0, and finishes at a desiresd time.
function desired_state_ = eval_trajectory(trajectory_generator_, t_)
  % Velocity is just finite differences in the trajectory

  pos = trajectory_generator_(t_);
  timestep = 0.02; % to calculate velocity
  pos_prev = trajectory_generator_(t_-timestep);
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

function trajparams = smooth_wp(waypoints)
    % todo
end

function desired = eval_traj_smooth(trajparams, t)
    % todo
end



end
