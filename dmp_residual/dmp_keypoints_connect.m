%% DMP with piece-wise linear function as elementary trajectory, inspired by VMP
% Model elementary trajectory, then use DMP to learn the residual (y-h).
% When adjust it to new via-points, f(x) stays fixed, simply modify h(x) according to new via-points.       
% * the frequency might be broken, but it's not a problem, can be adjusted by resampling after motion retargeting.     
% no need for all samples, one is enough.
% %%%%% DMP now simply serves as a Constant... as an offset... simply recording the residual between y(x) and h(x) can also finish the job....     

% addpath('../vmp/m_fcts/');
addpath('../vmp/');


%% Get imitation trajectory and preprocess
% load data
file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
info = h5info(file_name);
%{
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
%}

% extract target group's data
target_group_name = 'fengren';
num_imitation_data = 1; %eval(['group_name_dict.', group_name]) - 1; % fengren: eliminate the first bad sample
num_resampled_points = 50; %100;

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


%% Extract keypoints from the fisrt sample
target_traj = original_traj(:, :, 1);
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


%% Model the piece-wise linear function
time_range = linspace(0, 1, num_resampled_points); % canonical time, internal clock
% get ready
via_points = original_traj(:, kp_list, 1); % get the keypoints' values from the first sample
pass_points = [original_traj(:, 1, 1), via_points, original_traj(:, end, 1)]; % DOF * (num_keypoints+2), i.e. keypoints plus start and goal
pass_time = time_range(kp_list);
% model the function
ele_traj_struct_all = elementary_trajectory_model(pass_points, pass_time, pos_and_glove_id, quat_id);

% compute h(x) sequence
h_seq = elementary_trajectory_calculate_sequence(ele_traj_struct_all, time_range, pos_and_glove_id, quat_id);

% display for debug: original trajectory and h(x)
%{
h_seq_test = h5read(file_name, '/fengren_1/tmp_hseq_noisy');
y_seq_test = h5read(file_name, '/fengren_1/tmp_yseq_noisy');

figure;
idx = [1,2,3];%[5,6,7]; %
plot3(original_traj(idx(1), :, 1), original_traj(idx(2), :, 1), original_traj(idx(3), :, 1), 'b-'); hold on; grid on;
% plot3(h_seq_test(idx(1), :), h_seq_test(idx(2), :), h_seq_test(idx(3), :), 'g-');
% plot3(h_seq(idx(1), :), h_seq(idx(2), :), h_seq(idx(3), :), 'r-');
plot3(y_seq_test(idx(1), :), y_seq_test(idx(2), :), y_seq_test(idx(3), :), 'r-');

for v = 1 : size(kp_list, 2)+2
    plot3(pass_points(idx(1), v), pass_points(idx(2), v), pass_points(idx(3), v), 'g*'); % iterate to plot via-poitns
end
xlabel('x'); ylabel('y'); zlabel('z'); 
%}


%% Store residual, or learn residual ???
y_seq = original_traj(:, :, 1);
f_seq = get_residual(y_seq, h_seq, pos_and_glove_id, quat_id);

% display for debug: original trajectory and h(x)
%{
figure;
subplot(3, 1, 1); plot(time_range, f_seq(1, :)); grid on;
subplot(3, 1, 2); plot(time_range, f_seq(2, :)); grid on;
subplot(3, 1, 3); plot(time_range, f_seq(3, :)); grid on;
%}

% store the result
% save([target_group_name, '_tmp.mat'], 'f_seq', 'pos_and_glove_id', 'quat_id', 'time_range', 'pass_time');

h5create(file_name, ['/', group_name, '/keypoint_list'], size(kp_list));
h5write(file_name, ['/', group_name, '/keypoint_list'], kp_list);

h5create(file_name, ['/', group_name, '/pos_and_glove_id'], size(pos_and_glove_id));
h5write(file_name, ['/', group_name, '/pos_and_glove_id'], pos_and_glove_id);

h5create(file_name, ['/', group_name, '/quat_id'], size(quat_id));
h5write(file_name, ['/', group_name, '/quat_id'], quat_id);

h5create(file_name, ['/', group_name, '/time_range'], size(time_range));
h5write(file_name, ['/', group_name, '/time_range'], time_range); % for locating keypoints' time

h5create(file_name, ['/', group_name, '/pass_time'], size([0, pass_time, 1]));
h5write(file_name, ['/', group_name, '/pass_time'], [0, pass_time, 1]); % keypoints' time

h5create(file_name, ['/', group_name, '/pass_points'], size(pass_points));
h5write(file_name, ['/', group_name, '/pass_points'], pass_points); % initial keypoints' values

h5create(file_name, ['/', group_name, '/f_seq'], size(f_seq));
h5write(file_name, ['/', group_name, '/f_seq'], f_seq); % residual


% get a noisy sample
%{
pass_points_noise = pass_points;
a = 0; b = 0.01;
noise_pos = a + (b-a) * rand([3, size(pass_points_noise, 2)]);
pass_points_noise(1:3, :) = pass_points_noise(1:3, :) + noise_pos;
h5create(file_name, ['/', group_name, '/pass_points_noise'], size(pass_points_noise));
h5write(file_name, ['/', group_name, '/pass_points_noise'], pass_points_noise); % residual
%}


%% Try Adapt to new via-points (Above code should stay in MATLAB, below code is for test only, adaptation code is in c++(ubuntu))
% add noise
pass_points_new = pass_points;
a = 0; b = 0.01;
% noise_pos = a + (b-a) * rand([3, size(pass_points_new, 2)]);
% pass_points_new(1:3, :) = pass_points_new(1:3, :) + noise_pos;
noise_pos = a + (b-a) * rand([3, size(pass_points_new, 2)-2]);
% pass_points_new(1:3, 2:end-1) = pass_points_new(1:3, 2:end-1) + noise_pos;
pass_points_new(11:13, 2:end-1) = pass_points_new(11:13, 2:end-1) + noise_pos;

% model the function
ele_traj_struct_new = elementary_trajectory_model(pass_points_new, pass_time, pos_and_glove_id, quat_id);

% compute h(x) sequence
h_new = elementary_trajectory_calculate_sequence(ele_traj_struct_new, time_range, pos_and_glove_id, quat_id);

% get y(x)
% y_new = h_new + f_seq; % ignore quaternion dimension for now...
y_new = get_final(f_seq, h_new, pos_and_glove_id, quat_id);

% display
%{
idx = [11,12,13];%[quat_id(1),quat_id(2),quat_id(3)];
figure;
plot3(y_seq(idx(1), :), y_seq(idx(2), :), y_seq(idx(3), :), 'b-'); hold on; grid on; % original trajectory
for v = 1 : size(kp_list, 2)+2
    plot3(pass_points(idx(1), v), pass_points(idx(2), v), pass_points(idx(3), v), 'ro'); % iterate to plot via-poitns
end
plot3(y_new(idx(1), :), y_new(idx(2), :), y_new(idx(3), :), 'g-');  % new trajectory
for v = 1 : size(kp_list, 2)+2
    plot3(pass_points_new(idx(1), v), pass_points_new(idx(2), v), pass_points_new(idx(3), v), 'g*'); % iterate to plot new via-poitns
end
xlabel('x'); ylabel('y'); zlabel('z'); 


% for presentation
title('fengren\_1, l\_wrist\_pos');

title('fengren\_1, r\_wrist\_pos');

view(-120, 45); % view the start (l_wrist_pos)

view(-120, 30); % view the mid (r_wrist_pos)
%}




