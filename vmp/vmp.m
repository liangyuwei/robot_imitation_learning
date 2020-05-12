%% Via-point movement primitive
% Process: learn VMP parameters from imitation trajectory -> adapt to given via-points           


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
    group_name = [target_group_name, '_', num2str(traj_id)];
    
    disp(['>>>> ---- processing ', group_name, ' ...']);
    
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


%% Learn VMP (adjusted from PbDlib, demo_proMP01.m)
addpath('./m_fcts/');

% calculate for 1-D trajectory, then iterate through all DOF
% quaternion is a little special, h(x) should be 4-dim coupled(for SLERP), while f(x) is 3-dim(q_w is calculated from unit quaternion constraint)
time_range = linspace(0, 1, num_resampled_points); % canonical time, internal clock

y_seq = original_traj(pos_and_glove_id(1), :, :); % all samples of the same dimension % (quat_id(1:4), :, :); %
% quat_id(1:4), quat_id(5:end)

ele_traj_struct = struct;
ele_traj_struct.floor = [0, 1]; % 1 is added for locating via-points closer to the end (by find function)
% ele_traj_struct.start = y_seq(:, 1, 1); ele_traj_struct.goal = y_seq(:, end, 1); % set from the first imitation data, directly connects the start and the end
% if pos/angle
ele_traj_struct.coeff = zeros(1, 6); % coefficients of fifth-order polynomial
% if quaternion
%{
ele_traj_struct.quat = zeros(2,4);
ele_traj_struct.quat(1, :) = y_seq(:, 1, 1)'; 
ele_traj_struct.quat(2, :) = y_seq(:, end, 1)';  % row vector
%}

% get shape modulation f(x) data for estimating the probability distribution of w (For pos, y(x) = h(x) + f(x); For quaternion, y(x) = h(x).*f(x))   
h_seq = zeros(size(y_seq));
f_seq = zeros(size(y_seq));
% if pos or angle
%
for d = 1 : num_imitation_data
    h_seq(:, :, d) = linspace(y_seq(1), y_seq(end), num_resampled_points); % since h(x) directly connects the start and end at the beginning, use linspace here as time_range does
end
f_seq = y_seq - h_seq;
%}
% if quaternion
%{
for d = 1 : num_imitation_data
    % get the start and goal of the current sample
    h_0 = y_seq(:, 1, d)'; h_1 = y_seq(:, end, d)';
    % iterate to get h_seq
    for t = 1 : size(time_range, 2)
        % SLERP interpolation
        h_seq(:, t, d) = quatinterp(h_0, h_1, time_range(t), 'slerp')';
        f_seq(:, t, d) = quatmultiply(y_seq(:, t, d)', quatinv(h_seq(:, t, d)'))';
    end
end
%}

% call function to estimate mu_w and sigma_w
nbStates = 8; % Number of basis functions
nbVar = size(f_seq, 1); % Dimension of position data (here: x1,x2), keep as 1 !!! % 1-dim for pos or angle data, 4-dim for quaternion
nbSamples = num_imitation_data; % Number of demonstrations
nbData = num_resampled_points; % Number of datapoints in a trajectory
traj_samples = f_seq; % All demonstration samples, should be of the size (DOF, nbData, nbSamples)

% for pos/angle, m is 1-dim; for quaternion, m is 3-dim(the real part is calculated from unit quaternion constraint)
m = proMP_get_mu_sigma_combine(nbStates, nbVar, nbSamples, nbData, traj_samples, time_range);

mu_w = m.Mu_w;
sigma_w = m.Sigma_w;


sigma_y = 0.005;
f_seq = shape_modulation_compute(m, time_range, sigma_y, false);



%% This function only computes f(x) values.

%% Extract keypoints from the learned model
% get the modeled trajectory



%% %%%%%%%%%%%% The following code better be included in reproduction phase
%% Morph the trajectories according to the given via-points (the number of keypoints is fixed during optimization, but the values at keypoints will be changed)
% Get via-points



% Adjust the trajectory according to the given via-points


















