function kp_indices = extract_joint_keypoints(joint_path, eps1, eps2, eps3, eps4)
%% This function extracts keypoints in joint path.
% joint_path - N * L, number of joints * length of path.

%% Preparation
DOF = size(joint_path, 1);
LEN = size(joint_path, 2);
kp_indices = [];
last_kp_id = 1; % the first as the initial keypoint, but not included

%% Iterate to determine
for i = 2:LEN % start with the second point...
    for j = 1:DOF % check all joints
        % condition 1
        if ( i - last_kp_id > eps1 && ...
             abs(joint_path(j, i)-joint_path(j, last_kp_id)) > eps2 && ...
             (joint_path(j, i) - joint_path(j, i-1))^2 < eps ) % vel->zero
            kp_indices = [kp_indices, i];        
            last_kp_id = i;
            break; % jump to next path point, no need to examine other joints
        end
        
        % condition 2
        if ( i - last_kp_id > eps4 && ...
             abs(joint_path(j, i) - joint_path(j, last_kp_id)) >= eps3)
            % check the points in-between
            n = last_kp_id + 1;
            kp_or_not = true;
            while( last_kp_id < n && n < i)
                if (abs(joint_path(j, n) - joint_path(j, last_kp_id)) >= eps3)
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

end
