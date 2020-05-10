%% Via-point movement primitive
% Process: learn VMP parameters from imitation trajectory -> adapt to given via-points           


%% Get imitation trajectory
file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
group_name = 'fengren_1';
l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']); % 3 x N
l_elbow_pos = h5read(file_name, ['/', group_name, '/l_elbow_pos']);
l_wrist_pos = h5read(file_name, ['/', group_name, '/l_wrist_pos']);
l_wrist_ori = h5read(file_name, ['/', group_name, '/l_wrist_ori']);  % 9 x N
r_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']);
r_elbow_pos = h5read(file_name, ['/', group_name, '/r_elbow_pos']);
r_wrist_pos = h5read(file_name, ['/', group_name, '/r_wrist_pos']);
r_wrist_ori = h5read(file_name, ['/', group_name, '/r_wrist_ori']);
l_glove_angle = h5read(file_name, ['/', group_name, '/l_glove_angle']); % 14 x N
r_glove_angle = h5read(file_name, ['/', group_name, '/r_glove_angle']);
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
    quat = rotm2quat(rotm);
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

original_traj = [l_wrist_pos; l_wrist_quat; l_elbow_pos; r_wrist_pos; r_wrist_quat; r_elbow_pos; l_glove_angle; r_glove_angle];
% 1,2,3;      4,5,6,7;      8, 9, 10;    11,12,13;    14,15,16,17;  18,19,20;


%% Learn VMP
y = h + f


y is 1-D !!!
choose to modify h or f according to conditional probability...
h(x) - piece-wise, minimum-jerk



%% Get via-points



%% Adjust the trajectory according to the given via-points


















