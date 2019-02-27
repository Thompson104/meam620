function finalpath = add_intermediate_points(path)
% if any points are too far apart, we want to add an intermediary point
    finalpath = path(1,:,:);
    for i = 2:length(path)
        prev_point = path(i-1,:,:);
        curr_point = path(i,:,:);
        dist = max(1e-3,norm(curr_point - prev_point));
        %if dist>maxlength
        if true
            % add an intermediary point
            % maybe we should do this for everything lol
            finalpath = [finalpath; 0.5*(curr_point+prev_point); curr_point];
        else
            finalpath = [finalpath; curr_point];
        end
    end
end
