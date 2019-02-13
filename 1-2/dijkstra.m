function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra

bounds = map.bound_xyz;
boundsvec = [bounds(1), bounds(4); bounds(2), bounds(5); bounds(3), bounds(6);];
mapsize = size(map.occgrid);
sub_bounds_vec = [pos2sub(map, boundsvec(:,1)), pos2sub(map, boundsvec(:,2))];

start_i = pos2ind(map, start);
goal_i = pos2ind(map, goal)
start_sub = ind2sub(size(map.occgrid), start_i);
goal_sub = ind2sub(size(map.occgrid), goal_i);
path = zeros(0,3);
num_expanded = 0;
is_not_in_bounds = any(goal<boundsvec(:,1) & goal>boundsvec(:,2) & start<boundsvec(:,1) & start>boundsvec(:,2));
is_in_obstacle = any(map.occgrid(start_sub) || map.occgrid(goal_sub)); %
if is_not_in_bounds
  disp('No Path found, start or goal not in bounds.');
  return
end

if is_in_obstacle
  disp('No Path found, start or goal in obstacle.');
  return
end

num_nodes = mapsize(1)*mapsize(2)*mapsize(3)
distance = inf(1, num_nodes); % All distances start at infinity
previous = -1*ones(1, num_nodes);
Q = 1:num_nodes;
distance(start_i) = 0; % distance to start is zero
distance_in_Q = distance;
while Q
  distances_in_Q = distance(Q);
  [u_dist,u] = min(distances_in_Q);
  Q(u) = [];
  num_expanded = num_expanded+1;

  [i,j,k] = ind2sub(size(map.occgrid), u);
  u_sub = [i,j,k];
  u_pos = ind2pos(map, u);
  all_neighbors = repmat(u_sub,6,1) + [eye(3); -eye(3)];

  for n = 1:6
    v_sub = all_neighbors(n,:);

    % make sure it's a valid neighbor first
    if any([v_sub > mapsize, v_sub<1])
      continue
    elseif map.occgrid(v_sub)
      % this neighbor is occupied
      continue
    end
    v_pos = sub2pos(map, v_sub);
    v_i = sub2ind(size(map.occgrid), v_sub);
    new_distance = u_dist + norm(u_pos - v_pos);
    if new_distance < distance(v_i)
      distance(v_i) = new_distance;
      previous(v_i) = u;
    end
  end
end

path_i = [];
u = goal_i;
while u ~= start_i
  if previous(u) ==-1
    disp('No path found. Problem infeasible.');
    return
  end
  path_i = [u,path_i];
  u = previous(u);
end
path = arrayfun(@(p) ind2pos(map, p)', path_i, 'Uniform', 0);
path = cell2mat(path)';

% gets neighbors which are in the map
% perturbation_subs = [eye(3); -eye(3)];  % precomputed
% function neighbors = get_neighbors(map, x_sub)
%   all_neighbors_sub = repmat(x_sub,6,1) + perturbation_subs;
%   % check to make sure in bounds
%   valid_neighbors_sub = all_neighbors_sub(all_neighbors_sub>sub_bounds_vec(:,1)' & all_neighbors_sub<sub_bounds_vec(:,2)');
%   % check to make sure not occupied
%
% end
%% Check whether things are in bounds or in an obstacle
