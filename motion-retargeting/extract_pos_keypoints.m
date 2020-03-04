function kp_indices = extract_pos_keypoints(pos_path, eps9, eps10, eps11, eps12, eps13, eps14, eps15)
%% This function extracts keypoints from position path.

%% Preparation
LEN = size(pos_path, 2);
kp_indices = [];
last_kp_id = 1; % the first as the initial keypoint, but not included


%% Iterate to determine
for i = 2:LEN-1 % start with the second point...
    % condition 1
    a = pos_path(:, i-1) - pos_path(:, i);
    b = pos_path(:, i+1) - pos_path(:, i);
    ang = acos(dot(a,b)/(norm(a)*norm(b))); % in radius
    if (ang < pi - eps9)
        kp_indices = [kp_indices, i];
        last_kp_id = i;
        continue; % jump to next path point, no need to examine other joints
    end
    
    % condition 2
    if ( norm(pos_path(:, i) - pos_path(:, i-1)) < eps10 && i - last_kp_id > eps11 && norm(pos_path(:, i) - pos_path(:, last_kp_id)) > eps12 )
        kp_indices = [kp_indices, i];
        last_kp_id = i;
        continue; % jump to next path point, no need to examine other joints
    end
    
    % condition 3
    if ( norm(pos_path(:,i) - pos_path(:, last_kp_id)) >= eps13 && i - last_kp_id > eps14 )
        % check the points in-between
        n = last_kp_id + 1;
        kp_or_not = true;
        while( last_kp_id < n && n < i)
            if ( norm(pos_path(:, n) - pos_path(:, last_kp_id)) >= eps15) % note that eps13 better be  greater than eps15
                kp_or_not = false; % not a keypoint since the condition is broken
            end
            n = n + 1;
        end
        if(kp_or_not)
            kp_indices = [kp_indices, i];
            last_kp_id = i;
            continue; % jump to next path point, no need to examine other joints
        end
    end
    
end

end