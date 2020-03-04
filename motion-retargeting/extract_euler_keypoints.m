function kp_indices = extract_euler_keypoints(euler_path, eps5, eps6, eps7, eps8)
%% This function extracts keypoints in joint path.
% euler_path - N * L, degree of freedom * length of path.

%% Preparation
DOF = size(euler_path, 1);
LEN = size(euler_path, 2);
kp_indices = [];
last_kp_id = 1; % the first as the initial keypoint, but not included

%% Iterate to determine
for i = 2:LEN % start with the second point...
    for j = 1:DOF % check all joints
        % condition 1
        if ( i - last_kp_id > eps5 && ...
             abs(euler_path(j, i)-euler_path(j, last_kp_id)) > eps6 && ...
             (euler_path(j, i) - euler_path(j, i-1))^2 < eps ) % vel->zero
            kp_indices = [kp_indices, i];        
            last_kp_id = i;
            break; % jump to next path point, no need to examine other joints
        end
        
        % condition 2
        if ( i - last_kp_id > eps8 && ...
             abs(euler_path(j, i) - euler_path(j, last_kp_id)) >= eps7)
            % check the points in-between
            n = last_kp_id + 1;
            kp_or_not = true;
            while( last_kp_id < n && n < i)
                if (abs(euler_path(j, n) - euler_path(j, last_kp_id)) >= eps7)
                    kp_or_not = false; % not a keypoint since the condition is broken
                end
                n = n + 1;
            end
            if (kp_or_not) % if all the conditions hold
                kp_indices = [kp_indices, i];
                last_kp_id = i;
                break; % jump to next path point, no need to examine other joints
            end     
        end
    end
end