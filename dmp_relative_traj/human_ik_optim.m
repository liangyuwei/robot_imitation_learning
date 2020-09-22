%% This script performs Human IK on motion capture data, i.e. convert motion capture position(or maybe orientation) trajectories to 7-DOF joint trajectories.
% Using optimization to solve IK for more precise results!!!

%% Prep
file_name = '../motion-retargeting/test_imi_data.h5'; % the original demonstrations that are not transformed to YuMi's local frames
group_name = 'fengren_1';


%% Load motion capture data
% position
l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_up_pos']);
r_shoulder_pos = h5read(file_name, ['/', group_name, '/r_up_pos']);
l_elbow_pos = h5read(file_name, ['/', group_name, '/l_fr_pos']);
r_elbow_pos = h5read(file_name, ['/', group_name, '/r_fr_pos']);
l_wrist_pos = h5read(file_name, ['/', group_name, '/l_hd_pos']);
r_wrist_pos = h5read(file_name, ['/', group_name, '/r_hd_pos']);

% quaternion, in (x,y,z,w) form
l_wrist_quat = h5read(file_name, ['/', group_name, '/l_hd_quat']);
r_wrist_quat = h5read(file_name, ['/', group_name, '/r_hd_quat']);
l_elbow_quat = h5read(file_name, ['/', group_name, '/l_fr_quat']);
r_elbow_quat = h5read(file_name, ['/', group_name, '/r_fr_quat']);

num_datapoints = size(l_wrist_pos, 2);

% convert quaternions to rotation matrices
l_wrist_rot = quat2rotm([l_wrist_quat(4, :); l_wrist_quat(1:3, :)]');
r_wrist_rot = quat2rotm([r_wrist_quat(4, :); r_wrist_quat(1:3, :)]');
l_elbow_rot = quat2rotm([l_elbow_quat(4, :); l_elbow_quat(1:3, :)]');
r_elbow_rot = quat2rotm([r_elbow_quat(4, :); r_elbow_quat(1:3, :)]');


%% Calculate 7-DOF joint angles from mocap data
l_joint_angle = zeros(7, num_datapoints);
r_joint_angle = zeros(7, num_datapoints);

% optimization setup
ub = [pi, pi/2, pi*2/3, pi*2/3, pi*2/3, pi/2, pi/4];
lb = -ub; % the same as human model urdf setting
x0_l = zeros(7, 1); % initial value
x0_r = zeros(7, 1); % 
global shoulder_pos upperarm_length forearm_length left_or_right
global elbow_pos_goal wrist_pos_goal wrist_rot_goal

% iterate to perform FK for each path point
l_cost_history = zeros(num_datapoints, 1);
r_cost_history = zeros(num_datapoints, 1);
for n = 1 : num_datapoints
    % debug information
    disp(['>>>> Tracking path point ', num2str(n), '/', num2str(num_datapoints), '...']);
    
    % 1 - Left
    disp('>> Left arm...');
    % get goals
    shoulder_pos = l_shoulder_pos(:, n);
    elbow_pos_goal = l_elbow_pos(:, n);
    wrist_pos_goal = l_wrist_pos(:, n);
    wrist_rot_goal = l_wrist_rot(:, :, n);
    upperarm_length = norm(shoulder_pos - elbow_pos_goal);
    forearm_length = norm(elbow_pos_goal - wrist_pos_goal);
    left_or_right = true;
    % start optimization
    [xl, fl] = fmincon(@human_ik_cost, x0_l, [], [], [], [], lb, ub);
    x0_l = xl;
    l_cost_history(n) = fl;
    % store the result
    l_joint_angle(:, n) = xl;
    
    % 2 - Right
    disp('>> Right arm...');
    % get goals
    shoulder_pos = r_shoulder_pos(:, n);
    elbow_pos_goal = r_elbow_pos(:, n);
    wrist_pos_goal = r_wrist_pos(:, n);
    wrist_rot_goal = r_wrist_rot(:, :, n);
    upperarm_length = norm(shoulder_pos - elbow_pos_goal);
    forearm_length = norm(elbow_pos_goal - wrist_pos_goal);
    left_or_right = false;
    % start optimization
    [xr, fr] = fmincon(@human_ik_cost, x0_r, [], [], [], [], lb, ub);
    x0_r = xr;
    r_cost_history(n) = fr;
    % store the result
    r_joint_angle(:, n) = xr;
end


%% Cost history ?
figure;
plot(1:num_datapoints, l_cost_history, 'b-'); hold on; grid on;
plot(1:num_datapoints, r_cost_history, 'r-');
title('Cost history');
xlabel('Path point'); ylabel('Costs');

%% Store the human IK results into file
h5create(file_name, ['/', group_name, '/l_joint_angles_optim_ik'], size(l_joint_angle));
h5write(file_name, ['/', group_name, '/l_joint_angles_optim_ik'], l_joint_angle);

h5create(file_name, ['/', group_name, '/r_joint_angles_optim_ik'], size(r_joint_angle));
h5write(file_name, ['/', group_name, '/r_joint_angles_optim_ik'], r_joint_angle);


