%% visualize the optimized pass points

groupname = 'fengren_1';

%% Prepare f_seq and others
filename1 = '../motion-retargeting/test_imi_data_YuMi.h5';

f_seq = h5read(filename1, ['/', groupname, '/f_seq']);
pos_and_glove_id = h5read(filename1, ['/', groupname, '/pos_and_glove_id']);
quat_id = h5read(filename1, ['/', groupname, '/quat_id']);
pass_points_ori = h5read(filename1, ['/', groupname, '/pass_points']);

num_datapoints = size(f_seq, 2);

time_range = linspace(0, 1, num_datapoints);

pass_time = h5read(filename1, ['/', groupname, '/pass_time']);
pass_time = pass_time(2:end-1); % eliminate the start and final point for ease of modeling h_seq

%% Load the optimized pass points
filename2 = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';

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
idx = [11,12,13]; %[5,6,7]; %
plot3(y_seq_ori(idx(1), :), y_seq_ori(idx(2), :), y_seq_ori(idx(3), :), 'b-'); hold on; grid on;
plot3(y_seq_new(idx(1), :), y_seq_new(idx(2), :), y_seq_new(idx(3), :), 'r-');

for v = 1 : size(kp_list, 2)+2
    plot3(pass_points_new(idx(1), v), pass_points_new(idx(2), v), pass_points_new(idx(3), v), 'g*'); % iterate to plot via-poitns
    plot3(pass_points_ori(idx(1), v), pass_points_ori(idx(2), v), pass_points_ori(idx(3), v), 'g*'); % iterate to plot via-poitns    
end
xlabel('x'); ylabel('y'); zlabel('z'); 

