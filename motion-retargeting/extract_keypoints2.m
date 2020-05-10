%% extract_keypoints.m


%% Load data from mocap data file
file_name = 'test_imi_data_YuMi.h5';
group_name = 'fengren_1';
info = h5info(file_name);
l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']); 
l_elbow_pos = h5read(file_name, ['/', group_name, '/l_elbow_pos']); 
l_wrist_pos = h5read(file_name, ['/', group_name, '/l_wrist_pos']); 
l_wrist_ori = h5read(file_name, ['/', group_name, '/l_wrist_ori']); 

r_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']); 
r_elbow_pos = h5read(file_name, ['/', group_name, '/r_elbow_pos']); 
r_wrist_pos = h5read(file_name, ['/', group_name, '/r_wrist_pos']); 
r_wrist_ori = h5read(file_name, ['/', group_name, '/r_wrist_ori']); 

l_glove_angle = h5read(file_name, ['/', group_name, '/l_glove_angle']); 
r_glove_angle = h5read(file_name, ['/', group_name, '/r_glove_angle']); 

length = size(l_wrist_pos, 2);

% convert rotation matrices to euler angles
l_wrist_euler = zeros(3, length);
r_wrist_euler = zeros(3, length); % in radius
for i = 1:length
    % left
    rotm = reshape(l_wrist_ori(:, i), 3, 3);
    rotm = rotm';
    eul = rotm2eul(rotm);
    l_wrist_euler(:, i) = eul';
    % right
    rotm = reshape(r_wrist_ori(:, i), 3, 3);
    rotm = rotm';
    eul = rotm2eul(rotm);
    r_wrist_euler(:, i) = eul';    
end



%% Extract keypoints and store
% joint keypoints conditions' threshold values
eps1 = 4; % sufficient time passed
eps2 = 45 * pi / 180; % enough amount of change in angles (in radius), since last keypoint; Intuition: reach extremum or stop changing.
eps3 = 5 * pi / 180; % enough amount of change in angles (in radius), since last keypoint; Intuition: start changing.
eps4 = 6; % sufficient time passed

% euler keypoints conditions' threshold values
eps5 = 3; % sufficient time passed
eps6 = 5 * pi / 180; % enough amount of change in angles (in radius); Intuition: reach extremum or stop changing.
eps7 = 5 * pi / 180; % amount of change in angles (in radius); Intuition: start changing.
eps8 = 3; % sufficient time passed

% position keypoints conditions' threshold values
eps9 = 80 * pi / 180; % set high enough to detect sharp cornors, in radius

eps10 = 0.02; % position vector magnitude change from last point; in meter
eps11 = 2; % sufficient time passed since last keypoint
eps12 = 0.03; % enough amount of change in position vector magnitude since last keypoint, in meter

eps13 = 0.02; % enough amount of change in position vector magnitude since last keypoint
eps14 = 2; % sufficient time passed
eps15 = 0.02;  % condition is valid only when eps13 is greater than or equal to eps15

% extract keypoints
joint_angle_path = [l_glove_angle; r_glove_angle];
euler_path = [l_wrist_euler; r_wrist_euler];
pos_path = [l_wrist_pos; l_elbow_pos; r_wrist_pos; r_elbow_pos];
DOF = size(euler_path, 1);
LEN = size(euler_path, 2);
kp_indices = [];
last_kp_id = 1; % the first as the initial keypoint, but not included
for i = 2:LEN-1 % start with the second point...
    %% check joint angle path
    for j = 1:DOF % check all joints
        % condition 1
        if ( i - last_kp_id > eps1 && ...
             abs(joint_angle_path(j, i)-joint_angle_path(j, last_kp_id)) > eps2 && ...
             (joint_angle_path(j, i) - joint_angle_path(j, i-1))^2 < eps ) % vel->zero
            kp_indices = [kp_indices, i];        
            last_kp_id = i;
            break; % jump to next path point, no need to examine other joints
        end
        
        % condition 2
        if ( i - last_kp_id > eps4 && ...
             abs(joint_angle_path(j, i) - joint_angle_path(j, last_kp_id)) >= eps3)
            % check the points in-between
            n = last_kp_id + 1;
            kp_or_not = true;
            while( last_kp_id < n && n < i)
                if (abs(joint_angle_path(j, n) - joint_angle_path(j, last_kp_id)) >= eps3)
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
    
    %% Check orientation
    if (last_kp_id == i)
        continue;
    end
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
    
    %% Check position
    if (last_kp_id == i)
        continue;
    end
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




%% Display the results
%{
target = 'r_wrist_pos';
ori_path = eval(target); % l_wrist_ori
kp_id_list = eval([target, '_kp_id_list']);

figure;
plot3(ori_path(1, :), ori_path(2, :), ori_path(3, :), 'b-'); grid on; hold on;
for i = 1 : size(kp_id_list, 2)
    plot3(ori_path(1, kp_id_list(i)), ori_path(2, kp_id_list(i)), ori_path(3, kp_id_list(i)), 'rx');
end
xlabel('x'); ylabel('y'); zlabel('z');
title('Keypoints, extracted from pos/ori/angles trajectories, no overlapping');
%}

%% Output constraints
%
kp_id_list = unique([l_wrist_pos_kp_id_list, r_wrist_pos_kp_id_list, ...
                     l_elbow_pos_kp_id_list, r_elbow_pos_kp_id_list, ...
                     l_wrist_ori_kp_id_list, r_wrist_ori_kp_id_list, ...
                     l_glove_angle_kp_id_list, r_glove_angle_kp_id_list]);

%{
h5create(file_name, ['/', group_name, '/kp_id_list'], size(kp_id_list)); % create before writing
h5write(file_name, ['/', group_name, '/kp_id_list'], kp_id_list);
%}


