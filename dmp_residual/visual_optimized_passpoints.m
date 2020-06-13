%% visualize the optimized pass points

groupname = 'fengren_1';

%% Prepare f_seq and others
filename1 = '../motion-retargeting/test_imi_data_YuMi.h5';

f_seq = h5read(filename1, ['/', groupname, '/f_seq']);
pos_and_glove_id = h5read(filename1, ['/', groupname, '/pos_and_glove_id']);
quat_id = h5read(filename1, ['/', groupname, '/quat_id']);
pass_points_ori = h5read(filename1, ['/', groupname, '/pass_points']);
kp_list = h5read(filename1, ['/', groupname, '/keypoint_list']);

num_datapoints = size(f_seq, 2);

time_range = linspace(0, 1, num_datapoints);

pass_time = h5read(filename1, ['/', groupname, '/pass_time']);
pass_time = pass_time(2:end-1); % eliminate the start and final point for ease of modeling h_seq

%% Load the optimized pass points
filename2 = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity-2.h5';

pass_points_new = h5read(filename2, ['/', groupname, '/passpoint_traj_1']);

% normalize the quaternion part of pass points...
for q_id = 1:2
    dof_id = quat_id((q_id-1)*4+1 : q_id*4);
    for n = 1 : size(pass_points_new, 2)
        pass_points_new(dof_id:dof_id+3, n) = quatnormalize(pass_points_new(dof_id:dof_id+3, n)')';
    end
end

%% Generate new trajectory based on the given optimized pass points
% model the function
ele_traj_struct_all_new = elementary_trajectory_model(pass_points_new, pass_time, pos_and_glove_id, quat_id);

% compute h(x) sequence
h_seq_new = elementary_trajectory_calculate_sequence(ele_traj_struct_all_new, time_range, pos_and_glove_id, quat_id);

% compute y(x) sequence
y_seq_new = get_final(f_seq, h_seq_new, pos_and_glove_id, quat_id);


%% Generate the original trajectory
% model the function
ele_traj_struct_all_ori = elementary_trajectory_model(pass_points_ori, pass_time, pos_and_glove_id, quat_id);

% compute h(x) sequence
h_seq_ori = elementary_trajectory_calculate_sequence(ele_traj_struct_all_ori, time_range, pos_and_glove_id, quat_id);

% compute y(x) sequence
y_seq_ori = get_final(f_seq, h_seq_ori, pos_and_glove_id, quat_id);


%% display the results
figure;
idx = [1,2,3]; %[5,6,7]; %
plot3(y_seq_ori(idx(1), :), y_seq_ori(idx(2), :), y_seq_ori(idx(3), :), 'b-'); hold on; grid on;
plot3(y_seq_new(idx(1), :), y_seq_new(idx(2), :), y_seq_new(idx(3), :), 'r-');

for v = 1 : size(kp_list, 2)+2
    plot3(pass_points_new(idx(1), v), pass_points_new(idx(2), v), pass_points_new(idx(3), v), 'go'); % iterate to plot via-poitns
    plot3(pass_points_ori(idx(1), v), pass_points_ori(idx(2), v), pass_points_ori(idx(3), v), 'go'); % iterate to plot via-poitns    
end
% title('Right Wrist Position');
% view(-45, 45);
title('Left Wrist Position');
view(-60, 60);
xlabel('x'); ylabel('y'); zlabel('z'); 


%% Plot the cost history
wrist_pos_cost_history = h5read(filename2, ['/', groupname, '/wrist_pos_cost_history']);
wrist_ori_cost_history = h5read(filename2, ['/', groupname, '/wrist_ori_cost_history']);
elbow_pos_cost_history = h5read(filename2, ['/', groupname, '/elbow_pos_cost_history']);
finger_cost_history = h5read(filename2, ['/', groupname, '/finger_cost_history']);
similarity_cost_history = h5read(filename2, ['/', groupname, '/similarity_cost_history']);
smoothness_cost_history = h5read(filename2, ['/', groupname, '/smoothness_cost_history']);
col_cost_history = h5read(filename2, ['/', groupname, '/col_cost_history']);
pos_limit_cost_history = h5read(filename2, ['/', groupname, '/pos_limit_cost_history']);

figure;
plot((1:size(wrist_pos_cost_history, 2))*10, sum(wrist_pos_cost_history), 'b-'); hold on; grid on;
plot((1:size(wrist_pos_cost_history, 2))*10, sum(wrist_pos_cost_history), 'bo');
title('History of wrist position cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(wrist_ori_cost_history, 2))*10, sum(wrist_ori_cost_history), 'b-'); hold on; grid on;
plot((1:size(wrist_ori_cost_history, 2))*10, sum(wrist_ori_cost_history), 'bo');
title('History of wrist orientation cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(elbow_pos_cost_history, 2))*10, sum(elbow_pos_cost_history), 'b-'); hold on; grid on;
plot((1:size(elbow_pos_cost_history, 2))*10, sum(elbow_pos_cost_history), 'bo');
title('History of elbow position cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(finger_cost_history, 2))*10, sum(finger_cost_history), 'b-'); hold on; grid on;
plot((1:size(finger_cost_history, 2))*10, sum(finger_cost_history), 'bo');
title('History of finger angle cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(similarity_cost_history, 2))*10, similarity_cost_history, 'b-'); hold on; grid on;
plot((1:size(finger_cost_history, 2))*10, similarity_cost_history, 'bo');
title('History of similarity cost');
xlabel('Iterations'); ylabel('Cost Value'); 



