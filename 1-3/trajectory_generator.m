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
z_speed_multiplier = 0.5;
dist_gain = 3.8;
too_close_to_obs = 0.2;
use_mat_splines = false;
flip_way = false; % do not turn true

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
    if flip_way
      flipped_untrimmed_waypoints_ = flipud(untrimmed_waypoints_);
      flipped_trim1_waypoints_ = trim_path_raytrace(map_, flipped_untrimmed_waypoints_, too_close_to_obs, dist_gain);

      % now we trim it the other way
      trim1_waypoints_ = flipud(flipped_trim1_waypoints_);
      sparse_waypoints_ = trim_path_raytrace(map_, trim1_waypoints_, too_close_to_obs, dist_gain);
    else
      flipped_untrimmed_waypoints_ = flipud(untrimmed_waypoints_);
      flipped_keep1_waypoints_ = keep_path_raytrace(map_, flipped_untrimmed_waypoints_, too_close_to_obs, dist_gain);

      % now we trim it the other way
      keep1_waypoints_ = flipud(flipped_keep1_waypoints_);
      keep2_waypoints_ = keep_path_raytrace(map_, untrimmed_waypoints_, too_close_to_obs, dist_gain);

      keep_waypoints = or(keep1_waypoints_, keep2_waypoints_);
      % trim one more time
      to_trim_waypoints = untrimmed_waypoints_(keep_waypoints,:,:);
      sparse_waypoints_ = trim_path_raytrace(map_, to_trim_waypoints, too_close_to_obs+0.05, dist_gain);

    end


  waypoints_ = add_intermediate_points(sparse_waypoints_);
  %waypoints_ = add_intermediate_points(waypoints_);
  [cumu_segment_lengths, t_f, total_path_length, slengths] = process_waypoints(waypoints_);

  trajectory_generator_straight = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length);
  trajectory_generator_spline = @(t) interp1(cumu_segment_lengths, waypoints_, max(min(t,t_f),0)/t_f*total_path_length, 'makima');

  % test
  if use_mat_splines
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
          trajectory_generator_dense = @(t) interp1(cumu_segment_lengths, dense_waypoints_, max(min(t,t_f),0)/t_f*total_path_length, 'makima');
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
  else
      % use the min snap stuff
      %evenly sample waypoints
      trajectory_generator_ = smooth_wp(waypoints_);
      t_f = trajectory_generator_.t0(end);
      test_t = 0:0.1:t_f;
      dense_waypoints_ = add_intermediate_points(waypoints_);
      trajectory_generator_ = smooth_wp(dense_waypoints_);
      
      for i=1:length(test_t)
        d = eval_traj_smooth(trajectory_generator_, test_t(i));
        ds(i,:) = d.pos;
      end
      spline_length = sum(vecnorm(diff(ds),2,2));
      fprintf('Total path length %f, total min jerk length %f \n', [total_path_length, spline_length]); 

  end


end

function [cumu_seg_len, t_fin, total_path_length, segment_lengths] = process_waypoints(wp)
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

    num_waypoints = length(waypoints);
    trajparams.start = waypoints(1,:);
    trajparams.goal = waypoints(num_waypoints,:);

    empt = zeros(1,length(trajparams.start));
    tstart = 0;
    qstart = waypoints(1,:);
    qend = waypoints(2,:);
    qdiff = qend - qstart;
    zang = 1;
    if qdiff(3)<0
        qmin = 0.2;
        qmax = 1;
        dzdx = max(qmin, min(qmax, norm(qdiff(3)) /min(1e-3, norm(qdiff(1:2)))));
        zang=z_speed_multiplier*dzdx/(qmax-qmin);
    end
    tend = tstart+norm(qend-qstart)/(target_speed*zang); % velocity continuity
    xvec = [trajparams.start; empt; empt; ];
    trajparams.t0 = tend;

    % we're just going to do continuous acceleration
    Qtot = [];
    Q1 = [1 0 0 0 0; 0 1 0 0 0; 0 0 2 0 0; ...
         1 tend tend^2 tend^3 tend^4;...
         0 0 0 0 0;...
         0 1 2*tend,3*tend^2, 4*tend^3;...
         0 0 2 6*tend 12*tend^2];
    Qtot = [Qtot, Q1];


    for j =2:num_waypoints-2
        qstart = waypoints(j,:);
        qend = waypoints(j+1,:);
        xvec = [xvec; qstart; qstart; empt;empt ];
        tstart = tend;
        
        qdiff = qend - qstart;
        zang = 1;
        if qdiff(3)<0
            qmin = 0.2;
            qmax = 1;
            dzdx = max(qmin, min(qmax, norm(qdiff(3)) /min(1e-3, norm(qdiff(1:2)))));
            zang=z_speed_multiplier*dzdx/(qmax-qmin);
        end
        tend = tstart+norm(qend-qstart)/(target_speed*zang); % velocity continuity
        %tend = tstart+norm(qend-qstart)/target_speed; % velocity

        Q = [1 tstart tstart^2 tstart^3; ...
         0 -1 -2*tstart  -3*tstart^2; ...
         0 0 -2 -6*tstart; ...
         1 tend tend^2 tend^3;...
         0 0 0 0;...
         0 1 2*tend,3*tend^2;...
         0 0 2 6*tend];


        ro = size(Q,1);
        co = size(Q,2);
        k = 4*(j-1)+1;
        Qtot(k:k+ro-1,k+1:k+co) = Q;

        trajparams.t0 = [trajparams.t0;tend];
    end

    % Do final step
    j = j+1;
    qstart = waypoints(j,:);
    qend = waypoints(j+1,:);
    xvec = [xvec; qstart; qstart; empt; empt];
    tstart = tend;
    
    qdiff = qend - qstart;
    zang = 1;

    if qdiff(3)<0
        qmin = 0.2;
        qmax = 1;
        dzdx = max(qmin, min(qmax, norm(qdiff(3)) /min(1e-3, norm(qdiff(1:2)))));
        zang=z_speed_multiplier*dzdx/(qmax-qmin);
    end
    speed_end = 0.7*target_speed;
    tend = tstart+norm(qend-qstart)/(speed_end*zang); % velocity continuity
    %tend = tstart+norm(qend-qstart)/target_speed; % velocity continuity
    trajparams.t0 = [trajparams.t0;tend];
    Qf = [1 tstart tstart^2 tstart^3 tstart^4; ...
     0 -1 -2*tstart  -3*tstart^2 -4*tstart^3; ...
     0 0 -2 -6*tstart -12*tstart^2; ...
     1 tend tend^2 tend^3 tend^4;...
     0 1 2*tend,3*tend^2, 4*tend^3;...
     0 0 2 6*tend 12*tend^2;];
    k = 4*(j-1)+1;
    Qtot(k:k+5,k+1:k+1+4) = Qf;
    xvec = [xvec; trajparams.goal; empt; empt;];


    trajparams.A = Qtot\xvec;

    % Debugging
    trajparams.Qtot = Qtot;
    trajparams.xvec = xvec;
end

function desired = eval_traj_smooth(trajparams, t)
    l = size(trajparams.t0,1);
    t = max(0.0, min(trajparams.t0(end), t));

    k = min(l,length(find(trajparams.t0<t))+1); % is the minimum since we should index at least the first value

    A = trajparams.A;
    z = zeros(1,length(A));
    pz = z;
    vz = z;
    az = z;

    if k == 1
        start_i = 1;
        end_i = start_i+4;
        pz(:,start_i:end_i) = [ 1, t, t^2, t^3, t^4, ];
        vz(:,start_i:end_i) = [0, 1, 2*t, 3*t^2, 4*t^3,];
        az(:,start_i:end_i) = [0, 0, 2, 6*t, 12*t*t, ];
    elseif k == l
        start_i = size(A,1)-4;
        end_i = start_i+4;
        pz(:,start_i:end_i) = [ 1, t, t^2, t^3, t^4, ];
        vz(:,start_i:end_i) = [0, 1, 2*t, 3*t^2, 4*t^3, ];
        az(:,start_i:end_i) = [0, 0, 2, 6*t, 12*t*t, ];
    else
        start_i = 6+(k-2)*4;
        end_i = start_i+3;
        pz(:,start_i:end_i) = [ 1, t, t^2, t^3, ];
        vz(:,start_i:end_i) = [0, 1, 2*t, 3*t^2, ];
        az(:,start_i:end_i) = [0, 0, 2, 6*t, ];
    end

    desired.pos = pz*A;
    desired.vel = vz*A;
    desired.acc = az*A;
    desired.pos = desired.pos';
    desired.vel = desired.vel';
    desired.acc = desired.acc';
    desired.yaw = 0;
    desired.yawdot = 0;
    desired.k = k;
    
end



end
