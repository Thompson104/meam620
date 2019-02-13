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

% create vertex set Q
%  4
%  5      for each vertex v in Graph:             // Initialization
%  6          dist[v] ← INFINITY                  // Unknown distance from source to v
%  7          prev[v] ← UNDEFINED                 // Previous node in optimal path from source
%  8          add v to Q                          // All nodes initially in Q (unvisited nodes)
%  9
% 10      dist[source] ← 0                        // Distance from source to source
% 11
% 12      while Q is not empty:
% 13          u ← vertex in Q with min dist[u]    // Node with the least distance
% 14                                              // will be selected first
% 15          remove u from Q
% 16
% 17          for each neighbor v of u:           // where v is still in Q.
% 18              alt ← dist[u] + length(u, v)
% 19              if alt < dist[v]:               // A shorter path to v has been found
% 20                  dist[v] ← alt
% 21                  prev[v] ← u
% 22
% 23      return dist[], prev[]
%
res = map.res_xyz;
% Finds valid neighbors of a function
% position
perturbation_positions = [eye(3); -eye(3)];  % precomputed
  function valid_neighbors = get_valid_neighbors(pos)
    sub = pos2sub(map, pos);
    all_neighbors = repmat(sub,6,1) + perturbation_positions;
    valid_neighbors = all_neighbors(all_neighbors>boundsvec(:,1)' & all_neighbors<boundsvec(:,2)');
  end

bounds = map.bound_xyz;
boundsvec = [bounds(1), bounds(4); bounds(2), bounds(5); bounds(3), bounds(6);];
path = zeros(0,3);
num_expanded = 0;
% Check to make sure points are within limits
is_not_in_bounds = any(goal<boundsvec(:,1) & goal>boundsvec(:,2) & start<boundsvec(:,1) & start>boundsvec(:,2));
is_not_in_obstacle = 0; % TODO
if is_not_in_bounds
  disp('No Path found, start or goal not in bounds.');
  return
end

if is_not_in_obstacle
  disp('No Path found, start or goal in obstacle.');
  return
end

% Set up Dijkstra's algorithm
x_range = bounds(4)- bounds(1);
y_range = bounds(5)- bounds(2);
z_range = bounds(6)- bounds(3);

% Important indexes
max_i = x_range/res(1)*y_range/res(2)*z_range/res(3);
start_i = pos2ind(map, start);

unvisited = 1:max_i;
distances = inf*ones(1, max_i);
distances(start_i) = 0;

previous_nodes = zeros(1, max_i);
previous_dist = 0;

% This should not be called until these are initialized
current_node_pos = [];
current_node_i = 0;
% TODO: turn this into a lambda function
function new_distance = update_neighbor(indd)
  disp(indd)
  % indd = unvisited_neighbors_i(j);
  % TODO: replace ind2pos with some smarter function
  new_distance = distances(current_node_i) + norm(current_node_pos - ind2pos(map, indd));
  if new_distance < distances(indd)
    distances(indd) = new_distance;
    previous_nodes(indd) = current_node_i;
  end
end

while ~isempty(unvisited)
  unvisited_distances = distances(unvisited);
  current_node_ = find(unvisited_distances==min(unvisited_distances));
  current_node_i = current_node_(1); % make sure current node is a single value
  current_node_pos = ind2pos(map, current_node_i);
  unvisited(current_node_i) = []; % remove current node from unvisited
  num_expanded = num_expanded + 1;
  neighbors_sub = get_valid_neighbors(current_node_pos);

  neighbors_i = arrayfun(@(p) sub2ind(size(map.occgrid),p), neighbors_sub);
  unvisited_neighbors_i = intersect(neighbors_i, unvisited);
  % Helper function to update the neighbor positions

  dump = arrayfun(@(p) update_neighbor(p), unvisited_neighbors_i);
end
j = pos2ind(map, goal)
if previous_nodes(j) == 0
  % we did not find a path :(
  disp('No Path found, problem infeasible');
  return
end
% otherwise we found a path!
while j ~= start_i
  path = horzcat(prev(j));
end

end
