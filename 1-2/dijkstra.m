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

res_x = map.res_xyz(1);
res_y = map.res_xyz(2);
res_z = map.res_xyz(3);
hwd = size(map.occgrid);
lo_x = map.bound_xyz(1);
lo_y = map.bound_xyz(2);
lo_z = map.bound_xyz(3);

if size(hwd, 2) < 3
    map.occgrid = zeros(pos2sub(map,map.bound_xyz(4:6)));
end
hwd = [size(map.occgrid, 1), size(map.occgrid, 2), size(map.occgrid, 3)];

ind_s = pos2ind(map, start);
ind_g = pos2ind(map, goal);
[gi, gj, gk] = ind2sub(size(map.occgrid), ind_g);

if map.occgrid(gi, gj, gk) == 1
    path = zeros(0, 3);
    num_expanded = 0;
    return
end

l1 = hwd(1);
l2 = hwd(2);
l3 = hwd(3);
length = l1*l2*l3;
g = zeros(1, length);
h = g;
finish = g;
g(:) = inf;
p = g;
g(ind_s) = 0;
if astar
    h(ind_s) = norm(start - goal);
end
f = g + h;

while any(f < inf) && finish(ind_g) == 0
    [~, u] = min(f);
    f(u) = inf;   
    finish(u) = 1;
    
    [i,j,k] = ind2sub(size(map.occgrid), u);
    u_pos = [lo_x + res_x*(j - 1), lo_y + res_y*(i - 1), lo_z + res_z*(k - 1)];

    for m = -1:1
        new_i = i + m;
        if new_i < 1 || new_i > hwd(1)
            continue
        end
        for n = -1:1
            new_j = j + n;
            if new_j < 1 || new_j > hwd(2)
                continue
            end
            for l = -1:1
                if m == 0 && n == 0 && l == 0
                    continue
                end
                new_k = k + l;
                if new_k < 1 || new_k > hwd(3)
                    continue
                end
                if map.occgrid(new_i, new_j, new_k) == 1
                    continue
                end
                new_ind = (new_k - 1)*hwd(1)*hwd(2) + (new_j - 1)*hwd(1) + new_i;
                if finish(new_ind) == 1
                    continue
                end
                new_pos = [u_pos(1) + n*res_x, u_pos(2) + m*res_y, u_pos(3) + l*res_z];
                d = g(u) + norm(u_pos - new_pos);
                if d < g(new_ind)
                    g(new_ind) = d;
                    p(new_ind) = u;
                    if astar
                        h(new_ind) = norm(new_pos - goal);
                    end
                    f(new_ind) = g(new_ind) + h(new_ind);
                end
            end
        end
    end
end
if finish(ind_g) == 0
    path = zeros(0, 3);
    num_expanded = 0;
else
    current_ind = ind_g;
    path = goal;
    while current_ind ~= ind_s
        current_ind = p(current_ind);
        [ci, cj, ck] = ind2sub(size(map.occgrid), current_ind);
        current = [lo_x + res_x*(cj - 1), lo_y + res_y*(ci - 1), lo_z + res_z*(ck - 1)];
        if current_ind == ind_s
            path = [start; path];
        else
            path = [current; path];
        end
    end
    num_expanded = size(path, 1);
end
end
