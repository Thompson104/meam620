function trimmed_path_xyz = trim_path(map, path_xyz, dist_gain)
% Trims path based on distance from obs heuristic
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
    current_pos = path_xyz(i,:,:);
    dist_prev_neighbor = norm(current_pos - neighbor_prev);
    % if it's far enough from the previous neighbor
    if dist_to_obs*dist_gain < dist_prev_neighbor
        trimmed_path_xyz = [trimmed_path_xyz; current_pos];
        neighbor_prev = current_pos;
    end
end

trimmed_path_xyz = [trimmed_path_xyz; path_xyz(length(path_xyz),:,:)];

end
