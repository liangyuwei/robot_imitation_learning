%% Inspect the imitation data for inspiration~~

%% Get imitation trajectory and preprocess
% load data
file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
info = h5info(file_name);

% extract group_names and number of imitation data    
pattern = '[/-_]+\d'; % from / to _ followed by a digit number
group_name_dict = struct; % with no fields
for i = 1:size(info.Groups)
    str = info.Groups(i).Name;
    id = regexp(str, pattern);
    group_name = str(2:id-1);
    if isfield(group_name_dict, group_name)
        eval(['group_name_dict.', group_name, ' = ', 'group_name_dict.', group_name, ' + 1;']);
    else
        eval(['group_name_dict.', group_name, ' = 1;']);
    end
end

% extract target group's data
target_group_name = 'fengren';
num_imitation_data = eval(['group_name_dict.', group_name]);
num_resampled_points = 100;

l_wrist_pos_resampled = zeros(3, num_resampled_points);
l_elbow_pos_resampled = zeros(3, num_resampled_points);
r_wrist_pos_resampled = zeros(3, num_resampled_points);
r_elbow_pos_resampled = zeros(3, num_resampled_points);
l_glove_angle_resampled = zeros(14, num_resampled_points);
r_glove_angle_resampled = zeros(14, num_resampled_points);
l_wrist_quat_resampled = zeros(4, num_resampled_points);
r_wrist_quat_resampled = zeros(4, num_resampled_points);

DOF = 3 * 4 + 4 * 2 + 14 * 2; % 4 pos(3) + 2 quat(4) + 2 glove(14)
original_traj = zeros(DOF, num_resampled_points, num_imitation_data);

disp(['>>>> Extract data from Group: ', target_group_name, ' ...']);
for traj_id = 1 : num_imitation_data  
    %% Load original trajectory
    group_name = [target_group_name, '_', num2str(traj_id)]; %(traj_id+1)];  % fengren: eliminate the first bad sample
    
    disp(['>>>> ---- processing ', group_name, ' ...']);
    
    l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']); % 3 x N
    l_elbow_pos = h5read(file_name, ['/', group_name, '/l_elbow_pos']);
    l_wrist_pos = h5read(file_name, ['/', group_name, '/l_wrist_pos']);
    l_wrist_ori = h5read(file_name, ['/', group_name, '/l_wrist_ori']);  % 9 x N
    r_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']);
    r_elbow_pos = h5read(file_name, ['/', group_name, '/r_elbow_pos']);
    r_wrist_pos = h5read(file_name, ['/', group_name, '/r_wrist_pos']);
    r_wrist_ori = h5read(file_name, ['/', group_name, '/r_wrist_ori']);
    l_glove_angle = h5read(file_name, ['/', group_name, '/l_glove_angle']) .* pi / 180;    % 14 x N, convert to radius
    r_glove_angle = h5read(file_name, ['/', group_name, '/r_glove_angle']) .* pi / 180;    % convert to radius
    
    time = h5read(file_name, ['/', group_name, '/time']);
    length = size(l_wrist_pos, 2);
    % convert rotation matrices to euler angles
    % euler angles discontinuous due to singularity!!!
    l_wrist_euler = zeros(3, length);
    r_wrist_euler = zeros(3, length); % in radius
    l_wrist_quat = zeros(4, length);
    r_wrist_quat = zeros(4, length);
    for i = 1:length
        % left
        rotm = reshape(l_wrist_ori(:, i), 3, 3);
        rotm = rotm';
        eul = rotm2eul(rotm);
        quat = rotm2quat(rotm); % [w, x, y, z]
        l_wrist_euler(:, i) = eul';
        l_wrist_quat(:, i) = quat';
        % right
        rotm = reshape(r_wrist_ori(:, i), 3, 3);
        rotm = rotm';
        eul = rotm2eul(rotm);
        quat = rotm2quat(rotm);
        r_wrist_euler(:, i) = eul';
        r_wrist_quat(:, i) = quat';
    end
    

    
    %% Resample and combine
    % position and angle data
    l_wrist_pos_resampled = resample_traj(time, l_wrist_pos, num_resampled_points, false);
    l_elbow_pos_resampled = resample_traj(time, l_elbow_pos, num_resampled_points, false);
    r_wrist_pos_resampled = resample_traj(time, r_wrist_pos, num_resampled_points, false);
    r_elbow_pos_resampled = resample_traj(time, r_elbow_pos, num_resampled_points, false);
    l_glove_angle_resampled = resample_traj(time, l_glove_angle, num_resampled_points, false);
    r_glove_angle_resampled = resample_traj(time, r_glove_angle, num_resampled_points, false);
    % orientation (quaternion)
    l_wrist_quat_resampled = resample_traj(time, l_wrist_quat, num_resampled_points, true);
    r_wrist_quat_resampled = resample_traj(time, r_wrist_quat, num_resampled_points, true);
    % combine all
    original_traj(:, :, traj_id) = [l_wrist_pos_resampled; l_wrist_quat_resampled; l_elbow_pos_resampled; 
                                    r_wrist_pos_resampled; r_wrist_quat_resampled; r_elbow_pos_resampled; 
                                    l_glove_angle_resampled; r_glove_angle_resampled];
                                  % 1,2,3;      4,5,6,7;      8, 9, 10;    
                                  % 11,12,13;    14,15,16,17;  18,19,20;      
    % quat is [w, x, y, z]
    
end
% indices to retrieve different parts
% use the same convention as the similarity training dataset, for convenience   
oritraj_id = sort(randperm(size(original_traj, 1)));
quat_id = [4,5,6,7, 14,15,16,17];
pos_id = [1,2,3, 8,9,10, 11,12,13, 18,19,20];
wrist_id = [1,2,3, 11,12,13];
elbow_id = [8,9,10, 18,19,20];
glove_id = setdiff(setdiff(oritraj_id, quat_id), pos_id);
pos_and_glove_id = sort([pos_id, glove_id]);

disp('done.');


%% Extract keypoints from the selected sample
target_traj = original_traj(:, :, 1); % select one sample from the stack
l_wrist_pos = target_traj(1:3, :);
l_wrist_quat = target_traj(4:7, :);
l_elbow_pos = target_traj(8:10, :);
r_wrist_pos = target_traj(11:13, :);
r_wrist_quat = target_traj(14:17, :);
r_elbow_pos = target_traj(18:20, :);
l_glove_angle = target_traj(21:34, :);
r_glove_angle = target_traj(35:end, :);
kp_list = extract_keypoints_from_sample(l_wrist_pos, l_wrist_quat, l_elbow_pos, r_wrist_pos, r_wrist_quat, r_elbow_pos, l_glove_angle, r_glove_angle);
% display for debug
%{
figure;
plot3(original_traj(11, :, 1), original_traj(12, :, 1), original_traj(13, :, 1), 'b-'); hold on; grid on;
for k = 1 : size(kp_list, 2)
    plot3(original_traj(11, kp_list(k), 1), original_traj(12, kp_list(k), 1), original_traj(13, kp_list(k), 1), 'ro');
end
%}


%% Plot
l_pos_traj = l_wrist_pos;
r_pos_traj = r_wrist_pos;
rel_pos_traj = r_pos_traj - l_pos_traj;
num_datapoints = size(l_pos_traj, 2);
figure;
plot3(rel_pos_traj(1, :), rel_pos_traj(2, :), rel_pos_traj(3, :), 'b-'); hold on; grid on;
plot3(rel_pos_traj(1, 1), rel_pos_traj(2, 1), rel_pos_traj(3, 1), 'go'); 
plot3(rel_pos_traj(1, end), rel_pos_traj(2, end), rel_pos_traj(3, end), 'ro'); 
for k = 1 : size(kp_list, 2)
    plot3(rel_pos_traj(1, kp_list(k), 1), rel_pos_traj(2, kp_list(k), 1), rel_pos_traj(3, kp_list(k), 1), 'ro');
end


figure;
num_keypoints = size(kp_list, 2);
subplot(3, 1, 1); plot(1:num_datapoints, rel_pos_traj(1, :)); grid on; hold on; title('x');
plot(kp_list, rel_pos_traj(1, kp_list), 'ro'); % label keypoints
subplot(3, 1, 2); plot(1:num_datapoints, rel_pos_traj(2, :)); grid on; hold on; title('y');
plot(kp_list, rel_pos_traj(2, kp_list), 'ro');
subplot(3, 1, 3); plot(1:num_datapoints, rel_pos_traj(3, :)); grid on; hold on; title('z');
plot(kp_list, rel_pos_traj(3, kp_list), 'ro');

