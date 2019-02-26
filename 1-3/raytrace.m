function [ newpath ] = raytrace(map, path)
    sample_spacing = 0.1;
    too_close_to_obs = 0.25;
    size(path)
    pl = length(path)
    occxyz_list = ind2pos(map,find(map.occgrid==1));
    newpath = zeros(pl, 3);
    newpath(1,:,:) = path(1,:,:);
    prev_point = path(1,:,:);
    new_path_size = 1;
    for i = 2:pl
        curr_point = path(i,:,:);
        dist = norm(curr_point - prev_point);
        k = 0:sample_spacing/dist:1; % sample points from 0 to 1
        % sample the line between them
        % make sure they're far enough away from obstacles
        sample_points = interp1([0,1], [prev_point; curr_point], k);
        nearest_row_list = dsearchn(occxyz_list,sample_points);
        nearest_pos_on_path = occxyz_list(nearest_row_list,:,:);
        path_distances_to_obstacle = vecnorm(nearest_pos_on_path - sample_points, 2,2);
        if any(path_distances_to_obstacle<too_close_to_obs)
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

   newpath = newpath(1:new_path_size,:,:);
end
