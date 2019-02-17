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

if nargin < 4
    astar = false;
end


mapsize = [size(map.occgrid, 1), size(map.occgrid, 2), size(map.occgrid, 3)];

path = zeros(0,3);
num_expanded = 0;

bounds = map.bound_xyz;
boundsvec = [bounds(1), bounds(4); bounds(2), bounds(5); bounds(3), bounds(6);];

res_x = map.res_xyz(1);
res_y = map.res_xyz(2);
res_z = map.res_xyz(3);

lb_x = map.bound_xyz(1);
lb_y = map.bound_xyz(2);
lb_z = map.bound_xyz(3);

start_i = pos2ind(map, start);
goal_i = pos2ind(map, goal);

start_sub = pos2sub(map, start);
goal_sub = pos2sub(map, goal);


function check = checkgrid(sub)
  check = map.occgrid(sub(1), sub(2), sub(3));
end

% Check whether the start/endpoints are valid
is_not_in_bounds = any(goal<=boundsvec(:,1) & goal>=boundsvec(:,2) & start<=boundsvec(:,1) & start>=boundsvec(:,2));
is_in_obstacle = any([checkgrid(start_sub); checkgrid(goal_sub)]); %
if is_not_in_bounds
  disp('No Path found, start or goal not in bounds.');
  return
end

if is_in_obstacle
  disp('No Path found, start or goal in obstacle.');
  return
end

num_nodes = mapsize(1)*mapsize(2)*mapsize(3);
distances = inf(1, num_nodes); % taxicab distance
heuristic = zeros(1, num_nodes); % heuristic for astar
visited = zeros(1, num_nodes); % 1 is visited, corresponds to index
previous = -ones(1, num_nodes); % Previous index in the path for a given node
distances(start_i) = 0;
neighbor_delta = [eye(3); -eye(3); ~eye(3); -~eye(3);]; % precomputed
if astar
    heuristic(start_i) = norm(start - goal);
end

unvisited_distances = distances + heuristic;

while any(unvisited_distances < inf) && visited(goal_i) == 0
    [~, u] = min(unvisited_distances);
    unvisited_distances(u) = inf;
    visited(u) = 1;
    num_expanded = num_expanded+1;
    [i,j,k] = ind2sub(mapsize, u);
    u_pos = [lb_x + res_x*(j - 1), lb_y + res_y*(i - 1), lb_z + res_z*(k - 1)];

    for z = 1:12
        delta = neighbor_delta(z,:);
        v_sub = [i,j,k] + delta;

        if any([v_sub > mapsize, v_sub<1])
            % make sure on the map
          continue
        % elseif checkgrid(v_sub
        elseif map.occgrid(v_sub(1), v_sub(2), v_sub(3))
          % this neighbor is occupied
          continue
        end

        v_pos = [u_pos(1) + delta(2)*res_x, u_pos(2) + delta(1)*res_y, u_pos(3) + delta(3)*res_z];
        v_i = (v_sub(3) - 1)*mapsize(1)*mapsize(2) + (v_sub(2) - 1)*mapsize(1) + v_sub(1);

        if visited(v_i)
            continue
        end
        new_distance = distances(u) + norm(u_pos - v_pos);

        if new_distance < distances(v_i)
            distances(v_i) = new_distance;
            previous(v_i) = u;
            if astar
                heuristic(v_i) = norm(v_pos - goal);
            end
            unvisited_distances(v_i) = distances(v_i) + heuristic(v_i);
        end
    end
end

if visited(goal_i) == 0
    disp('No path found. Problem infeasible');
    return
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
path_i = [start_i, path_i];
path = arrayfun(@(p) ind2pos(map, p)', path_i, 'Uniform', 0);
path = cell2mat(path)';
% redo the freakin start and end
path = [start; path(2:length(path)-1,:,:); goal];

end
