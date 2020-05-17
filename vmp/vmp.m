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
num_imitation_data = eval(['group_name_dict.', group_name]) - 1; % fengren: eliminate the first bad sample
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
    group_name = [target_group_name, '_', num2str(traj_id+1)];  % fengren: eliminate the first bad sample
    
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


%% Learn VMP (adjusted from PbDlib, demo_proMP01.m)
addpath('./m_fcts/');
ele_traj_struct_all = cell(3*4+14*2+2, 1); % pos 3*4, glove 14*2, quat 2
model_all = cell(3*4+14*2+2, 1); % pos 3*4, glove 14*2, quat 2(quat's model is 3-dim)
time_range = linspace(0, 1, num_resampled_points); % canonical time, internal clock
y_rep = zeros(size(original_traj, 1), size(original_traj, 2));

% %%%%%%%%%%%%%%%%%%%%%% 1 - pos or angle %%%%%%%%%%%%%%%%%%%%%%
for pg_id = 1 : size(pos_and_glove_id, 2)
    % calculate for 1-D trajectory, then iterate through all DOF
    % quaternion is a little special, h(x) should be 4-dim coupled(for SLERP), while f(x) is 3-dim(q_w is calculated from unit quaternion constraint)
    
    y_seq = original_traj(pos_and_glove_id(pg_id), :, :);
    
    ele_traj_struct = struct;
    ele_traj_struct.floor = [0, 1]; % 1 is added for locating via-points closer to the end (by find function)
    
    % IF use fifth-order polynomial
%     ele_traj_struct.start = y_seq(1, 1, 1); ele_traj_struct.goal = y_seq(1, end, 1); % set from the first imitation sample, directly connects the start and the end
%     ele_traj_struct.coeff = zeros(2, 6); % coefficients of fifth-order polynomial; the last one is not used, just to be in consistence with .floor length
    
    % IF use piece-wise linear segment
    ele_traj_struct.ab = zeros(2, 2); % a and b, the coefficients of the 1st-order linear function h(x) = a * x + b
    g = y_seq(1, end, 1); y0 = y_seq(1, 1, 1);
    ele_traj_struct.ab(1, :) = [g-y0, y0]; % initial: h(x) = (g-y0)*x + y0
    
    % get shape modulation f(x) data for estimating the probability distribution of w (For pos, y(x) = h(x) + f(x); For quaternion, y(x) = h(x).*f(x))
    h_seq = zeros(size(y_seq));
    f_seq = zeros(size(y_seq));
    for d = 1 : num_imitation_data
        h_seq(:, :, d) = linspace(y_seq(1, 1, d), y_seq(1, end, d), num_resampled_points); % since h(x) directly connects the start and end at the beginning, use linspace here as time_range does
    end
    f_seq = y_seq - h_seq;
    
    % call function to estimate mu_w and sigma_w
    nbStates = 16; % Number of basis functions
    nbVar = size(f_seq, 1); % Dimension of position data (here: x1,x2), keep as 1 !!! % 1-dim for pos or angle data, 4-dim for quaternion
    nbSamples = num_imitation_data; % Number of demonstrations
    nbData = num_resampled_points; % Number of datapoints in a trajectory
    traj_samples = f_seq; % All demonstration samples, should be of the size (DOF, nbData, nbSamples)
    
    % for pos/angle, m is 1-dim; for quaternion, m is 3-dim(the real part is calculated from unit quaternion constraint)
    model = proMP_get_mu_sigma_combine(nbStates, nbVar, nbSamples, nbData, traj_samples, time_range);
    
    
    % reproduce the trajectory
    sigma_y = 0;%0.00001; % for pos or angle data?
    y_x = vmp_compute(ele_traj_struct, model, time_range, sigma_y, false);
    
    % store the meta-data
    model_all{pg_id} = model;
    ele_traj_struct_all{pg_id} = ele_traj_struct;
    y_rep(pos_and_glove_id(pg_id), :) = y_x;
    
end

% %%%%%%%%%%%%%%%%%%%%%% 2 - quaternion %%%%%%%%%%%%%%%%%%%%%%
for q_id = 1:2
    % extract 4-dim quaternion data
    y_seq = original_traj(quat_id((q_id-1)*4+1 : q_id*4), :, :);
    
    ele_traj_struct = struct;
    ele_traj_struct.floor = [0, 1]; % 1 is added for locating via-points closer to the end (by find function)
    % ele_traj_struct.start = y_seq(:, 1, 1); ele_traj_struct.goal = y_seq(:, end, 1); % set from the first imitation data, directly connects the start and the end
    % if quaternion, set initial data
    ele_traj_struct.quat = zeros(2,4);
    ele_traj_struct.quat(1, :) = y_seq(:, 1, 1)';
    ele_traj_struct.quat(2, :) = y_seq(:, end, 1)';  % row vector
    
    % get shape modulation f(x) data for estimating the probability distribution of w (For pos, y(x) = h(x) + f(x); For quaternion, y(x) = h(x).*f(x))
    h_seq = zeros(size(y_seq));
    f_seq = zeros(size(y_seq));
    
    % if quaternion
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
    
    % call function to estimate mu_w and sigma_w
    nbStates = 8; % Number of basis functions
    nbVar = size(f_seq, 1); % Dimension of position data (here: x1,x2), keep as 1 !!! % 1-dim for pos or angle data, 4-dim for quaternion
    nbSamples = num_imitation_data; % Number of demonstrations
    nbData = num_resampled_points; % Number of datapoints in a trajectory
    traj_samples = f_seq; % All demonstration samples, should be of the size (DOF, nbData, nbSamples)
    
    % for pos/angle, m is 1-dim; for quaternion, m is 3-dim(the real part is calculated from unit quaternion constraint)
    model = proMP_get_mu_sigma_combine(nbStates, nbVar, nbSamples, nbData, traj_samples, time_range);
    
    % reproduce the data
    sigma_y = 0;%0.00001; % for pos or angle data?
    y_x = vmp_compute(ele_traj_struct, model, time_range, sigma_y, true);
    
    % store the meta-data
    model_all{size(pos_and_glove_id, 2)+q_id} = model;
    ele_traj_struct_all{size(pos_and_glove_id, 2)+q_id} = ele_traj_struct;
    y_rep(quat_id((q_id-1)*4+1 : q_id*4), :) = y_x;
    
end

% %%%%%%%%%%%%%%%%%%%%%% Save the results %%%%%%%%%%%%%%%%%%%%%%
tmp_file_name = [target_group_name, '_initial.mat'];
save(tmp_file_name, 'model_all', 'ele_traj_struct_all', 'y_rep');


% display for debug
%{
idx = [1,2,3];
figure;
for i = 1 : 1%nbSamples
    plot3(original_traj(idx(1), :, i), original_traj(idx(2), :, i), original_traj(idx(3), :, i), 'b-'); hold on; grid on;
end
plot3(y_rep(idx(1),:), y_rep(idx(2),:), y_rep(idx(3),:), 'r.');

figure;
subplot(3,1,1); plot(1:num_resampled_points, original_traj(idx(1), :, 1), 'b.'); hold on; grid on;
plot(1:num_resampled_points, y_rep(idx(1), :), 'r.');
subplot(3,1,2), plot(1:num_resampled_points, original_traj(idx(2), :, 1), 'b.'); hold on; grid on;
plot(1:num_resampled_points, y_rep(idx(2), :), 'r.');
subplot(3,1,3), plot(1:num_resampled_points, original_traj(idx(3), :, 1), 'b.'); hold on; grid on;
plot(1:num_resampled_points, y_rep(idx(3), :), 'r.');
%}

%% Extract keypoints from the fisrt sample (the learned model doesn't look well, exact shape is not preserved)
target_traj = original_traj(:, :, 1);
l_wrist_pos = target_traj(1:3, :);
l_wrist_quat = target_traj(4:7, :);
l_elbow_pos = target_traj(8:10, :);
r_wrist_pos = target_traj(11:13, :);
r_wrist_quat = target_traj(14:17, :);
r_elbow_pos = target_traj(18:20, :);
l_glove_angle = target_traj(21:34, :);% .* pi/180;
r_glove_angle = target_traj(35:end, :);% .* pi/180;
kp_list = extract_keypoints_from_sample(l_wrist_pos, l_wrist_quat, l_elbow_pos, r_wrist_pos, r_wrist_quat, r_elbow_pos, l_glove_angle, r_glove_angle);
% display for debug
%{
figure;
plot3(original_traj(11, :, 1), original_traj(12, :, 1), original_traj(13, :, 1), 'b-'); hold on; grid on;
for k = 1 : size(kp_list, 2)
    plot3(original_traj(11, kp_list(k), 1), original_traj(12, kp_list(k), 1), original_traj(13, kp_list(k), 1), 'ro');
end
%}


%% %%%%%%%%%%%% The following code better be included in reproduction phase
%% Morph the trajectories according to the given via-points (the number of keypoints is fixed during optimization, but the values at keypoints will be changed)
disp('Morph the trajectories...');
% Get via-points
via_points = original_traj(:, kp_list, 1); % get the keypoints' values from the first sample
% copy the structure and initial trajectory for iteration  
ele_traj_struct_adjust = ele_traj_struct_all;
model_adjust = model_all;
y_seq_adjust = y_rep;   
threshold_prob = 0.8; %Inf; %0.5;
% iterate to add via-points (update function structure according to the given via-points)
for v = 1 : size(kp_list, 2)
    disp(['Processing via-point ', num2str(v), '...']);
    % common time 
    x_via = time_range(kp_list(v));
    % pos/angle via-point
    for pg_id = 1 : size(pos_and_glove_id, 2)
        % get via-point value
        y_via = via_points(pos_and_glove_id(pg_id), v); % v-th via-point
        sigma_via = 1E-4; % 0 means must go through
        % compute conditional probability
        prob = conditional_probability(model_adjust{pg_id}, ele_traj_struct_adjust{pg_id}, x_via, y_via, sigma_via, time_range);
        if isnan(prob)
            prob = 100; % NaN because y_via == mu, and sigma = 0; happens when one dimension doesn't change
        end
        disp(['Conditional probability = ', num2str(prob)]);
     
        % update model structure and meta-data
        threshold_prob = Inf;
        if prob > threshold_prob
            % probability is big enough, may use shape modualtion
%             h_via = elementary_trajectory_compute(ele_traj_struct_adjust{pg_id}, x_via, false);
            h_via = elementary_trajectory_compute_linear(ele_traj_struct_adjust{pg_id}, x_via, false);

            model_adjust{pg_id} = shape_modulation_add_viapoint(model_adjust{pg_id}, x_via, h_via, y_via, sigma_via, time_range);
        else
            % probability is not big enough, shape modulation works poorly, modify elementary trajectory instead                                 
            y_seq = y_seq_adjust(pos_and_glove_id(pg_id), :); %y_rep(pos_and_glove_id(pg_id), :); % get the current y!!!
            sigma_y = 0;
            f_seq = shape_modulation_compute(model_adjust{pg_id}, time_range, sigma_y, false);
%             ele_traj_struct_adjust{pg_id} = elementary_trajectory_add_viapoint(ele_traj_struct_adjust{pg_id}, x_via, y_via, y_seq, f_seq, time_range, false);
            ele_traj_struct_adjust{pg_id} = elementary_trajectory_add_viapoint_linear(ele_traj_struct_adjust{pg_id}, x_via, y_via, y_seq, f_seq, time_range, false);

        end
        % update the final trajectory
        sigma_y = 0;
        y_seq_adjust(pos_and_glove_id(pg_id), :) = vmp_compute(ele_traj_struct_adjust{pg_id}, model_adjust{pg_id}, time_range, sigma_y, false);
    end
    
    % quaternion via-point
    for q_id = 1:2
        % according to the paper, it's more complicated to calculate conditional probability than just integrating via-points into elementart trajectory   
        y_via = via_points(quat_id((q_id-1)*4+1 : q_id*4), v);
        % update the model structure (elementary trajectory)
        y_seq = y_seq_adjust(quat_id((q_id-1)*4+1 : q_id*4), :); %y_rep(quat_id((q_id-1)*4+1 : q_id*4), :);
        sigma_y = 0; % must go through
        f_seq = shape_modulation_compute(model_adjust{size(pos_and_glove_id, 2)+q_id}, time_range, sigma_y, true);
        
%         ele_traj_struct_adjust{size(pos_and_glove_id, 2)+q_id} = ...
%                                     elementary_trajectory_add_viapoint(ele_traj_struct_adjust{size(pos_and_glove_id, 2)+q_id}, ...
%                                     x_via, y_via, y_seq, f_seq, time_range, true);
        ele_traj_struct_adjust{size(pos_and_glove_id, 2)+q_id} = ...
                                    elementary_trajectory_add_viapoint_linear(ele_traj_struct_adjust{size(pos_and_glove_id, 2)+q_id}, ...
                                    x_via, y_via, y_seq, f_seq, time_range, true);
                                
        % update the final trajectory
        y_seq_adjust(quat_id((q_id-1)*4+1 : q_id*4), :) = ...
                                    vmp_compute(ele_traj_struct_adjust{size(pos_and_glove_id, 2)+q_id}, ...
                                    model_adjust{size(pos_and_glove_id, 2)+q_id}, time_range, sigma_y, true);
    end
    
end

% display for debug
idx = [11,12,13];
figure;
plot3(y_seq_adjust(idx(1), :), y_seq_adjust(idx(2), :), y_seq_adjust(idx(3), :), 'g-'); hold on; grid on;
% figure;
for v = 1 : size(kp_list, 2)
    plot3(via_points(idx(1), :), via_points(idx(2), :), via_points(idx(3), :), 'ro'); hold on; grid on;
end
for i = 1 : 1%nbSamples
    plot3(original_traj(idx(1), :, i), original_traj(idx(2), :, i), original_traj(idx(3), :, i), 'b-');
end
xlabel('x'); ylabel('y'); zlabel('z'); 



title('fengren\_1, l\_wrist\_pos');

title('fengren\_1, r\_wrist\_pos');

view(-120, 45); % view the start (l_wrist_pos)

view(-120, 30); % view the mid (r_wrist_pos)



