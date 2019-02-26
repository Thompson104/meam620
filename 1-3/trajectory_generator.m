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
target_speed = 2; % Meters per second
dist_gain = 3.8;

if isempty(t)
  % we need to generate a trajectory
  traj = generate_trajectory(map, path);
else
  desired_state = eval_trajectory(traj, t);
end


%% Trajectory Generation function
% Returns a function that takes time t as an input and returns a position
% on the trajectory.
function trajectory_generator_ = generate_trajectory(map_, untrimmed_waypoints_)
  % waypoints_ = trim_path(map_, untrimmed_waypoints_);
  waypoints_ = raytrace(map_, untrimmed_waypoints_);
  segment_vectors = waypoints_(2:end,:) - waypoints_(1:end-1,:);
  % the length of each segment
  % this is row-wise norm
  segment_lengths = sqrt(diag(segment_vectors*segment_vectors'));
  total_path_length = sum(segment_lengths);
  cumu_segment_lengths = [0; cumsum(segment_lengths)];

  t_f = total_path_length/target_speed;
  %trajectory_generator_ = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length);
  trajectory_generator_ = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length, 'spline');
  figure(6)
  test_t = 0:0.1:t_f;
  genpath = trajectory_generator_(test_t);
  spline_length = sum(vecnorm(diff(genpath),2,2));
  fprintf('Total path length %f, total spline length %f \n', [total_path_length, spline_length]);
  plot_path(map, trajectory_generator_(test_t))

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


function trimmed_path_xyz = trim_path(map, path_xyz)
    occxyz_list = ind2pos(map,find(map.occgrid==1));

    % first find out the distance of each point from an obstacle
    % then go through each point. If the point is d away from the obstacle,
    % and l to previous neighbor, and l < k*d, then kill this point. Otherwise
    % keep the point.
    nearest_row_list = dsearchn(occxyz_list,path_xyz);
    nearest_pos_on_path = occxyz_list(nearest_row_list,:,:);
    % list of distances for each point on path
    path_distances_to_obstacle = vecnorm(nearest_pos_on_path - path_xyz, 2,2);

    %figure(5)
    %plot(path_distances_to_obstacle)

    trimmed_path_xyz = path_xyz(1,:,:);
    neighbor_prev = path_xyz(1,:,:);

    % we keep the start and end nodes
    for i = 2:length(path_distances_to_obstacle)-1
        dist_to_obs = path_distances_to_obstacle(i);
        % trim based on hard distance to obstacle
        % if greater than 0.7, trim, otherwise be careful
        current_pos = path_xyz(i,:,:);
        dist_prev_neighbor = norm(current_pos - neighbor_prev);
        % if it's far enough from the previous neighbor
        if dist_to_obs*dist_gain < dist_prev_neighbor
            trimmed_path_xyz = [trimmed_path_xyz; current_pos];
            neighbor_prev = current_pos;
        end

        % Trim based on distance from obstacle scaled
    %    dist_prev_neighbor = norm(current_pos - neighbor_prev);

        % if it's far enough from the previous neighbor
%        if dist_to_obs*dist_gain < dist_prev_neighbor
%            trimmed_path_xyz = [trimmed_path_xyz; current_pos];
%            neighbor_prev = current_pos;
%        end
%
    end

    trimmed_path_xyz = [trimmed_path_xyz; path_xyz(length(path_xyz),:,:)];
end

end
