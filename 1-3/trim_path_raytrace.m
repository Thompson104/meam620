function [ newpath ] = trim_path_raytrace(map, path, too_close_to_obs, dist_gain)
    sample_spacing = 0.1;
    pl = length(path);
    occxyz_list = ind2pos(map,find(map.occgrid==1));
    newpath = zeros(pl, 3);
    newpath(1,:,:) = path(1,:,:);
    prev_point = path(1,:,:);
    new_path_size = 1;
    maxlength = 2; % max path length
    for i = 2:pl
        curr_point = path(i,:,:);
        dist = max(1e-3,norm(curr_point - prev_point));

        k = 0:sample_spacing/dist:1; % sample points from 0 to 1
        % sample the line between them
        % make sure they're far enough away from obstacles
        sample_points = interp1([0,1], [curr_point; prev_point], k);
        nearest_row_list = dsearchn(occxyz_list,sample_points);
        nearest_pos_on_path = occxyz_list(nearest_row_list,:,:);
        path_distances_to_obstacle = vecnorm(nearest_pos_on_path - sample_points, 2,2);
        dist_to_obs = path_distances_to_obstacle(1);
        is_far_from_neighbor = dist_to_obs*dist_gain < dist; % metric for trimming
        is_too_close_to_obs = any(path_distances_to_obstacle<too_close_to_obs); % metric for ray tracing
        if is_too_close_to_obs
            % too close, so keep the point
            new_path_size = new_path_size+1;
            newpath(new_path_size,:,:) = curr_point;
            prev_point = curr_point;
        end
        % otherwise we can skip this point
   end

   if any(newpath(new_path_size,:,:) ~= path(pl,:,:))
       new_path_size = new_path_size+1;
       newpath(new_path_size,:,:) = path(pl,:,:);
   end

   newpath = newpath(1:new_path_size,:,:)

end
