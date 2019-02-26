close all;
clear all;
clc;
addpath(genpath('./'));
map = load_map('map1.txt', 0.5, 1.0, 0.25);
start = {[0.0, -4.5, 0.5]};
stop = {[8.0, 18.0, 3.0]};

margin = 0.25;


nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
    % convert path in xyz to ijk
    % todo: fix
    % pathijk_cell = arrayfun(@(p) pos2sub(map, p)', path{qn}, 'Uniform', 0);
    % pathijk = cell2mat(pathijk_cell);
    pathijk = []
    for i = 1:length(path{qn})
        s = pos2sub(map,path{qn}(i,:,:));
        pathijk = [pathijk; s];
    end 
    
    % convert occmap to list of indicies
    mapsize = [size(map.occgrid, 1), size(map.occgrid, 2), size(map.occgrid, 3)];
    [occ_i, occ_j, occ_k] = ind2sub(mapsize,find(map.occgrid==1));
    occijk_list = [occ_i, occ_j, occ_k];
    trimmed_path = trim_path(map, occijk_list, pathijk, path{qn});
    
end
if nquad == 1
    %plot_path(map, path{1});
    %plot_path(map, trimmed_path);
else
    % you could modify your plot_path to handle cell input for multiple robots
end

function trimmed_path_xyz = trim_path(map, occijk_list, path_i, path_xyz)
    % first find out the distance of each point from an obstacle
    % then go through each point. If the point is d away from the obstacle,
    % and l to previous neighbor, and l < k*d, then kill this point. Otherwise 
    % keep the point.
    nearest_row_list = dsearchn(occijk_list,path_i)
    % the nearest points, this is a list parallel to path_i
    nearest_points_on_path_ijk = occijk_list(nearest_row_list, :, :);
    %nearest_pos_on_path = arrayfun(@(p) sub2pos(map, p)', nearest_points_on_path_ijk, 'Uniform', 0);
    nearest_pos_on_path_ = arrayfun(@(p) ind2pos(map, p), nearest_row_list, 'Uniform', 0);
    nearest_pos_on_path = cell2mat(nearest_pos_on_path_);
    
    % list of distances for each point on path
    path_distances_to_obstacle = vecnorm(nearest_pos_on_path - path_xyz, 2,2);
    figure
    plot(path_distances_to_obstacle)
    
    trimmed_path_xyz = path_xyz(1,:,:);
    neighbor_prev = path_xyz(1,:,:);
    dist_gain = 0.2;
    
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