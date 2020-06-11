%% visualize the optimized pass points

groupname = 'fengren_1';

%% Prepare f_seq and others
filename1 = '/home/liangyuwei/sign_language_robot_ws/test_imi_data/test_imi_data_YuMi.h5';

f_seq = h5read(filename1, ['/', groupname, '/f_seq']);
pos_and_glove_id = h5read(filename1, ['/', groupname, '/pos_and_glove_id']);
quat_id = h5read(filename1, ['/', groupname, '/quat_id']);
pass_time = h5read(filename1, ['/', groupname, '/pass_time']);
pass_points_ori = h5read(filename1, ['/', groupname, '/pass_points']);

num_datapoints = size(f_seq, 2);

time_range = linspace(0, 1, num_datapoints);


%% Load the optimized pass points
filename2 = '/home/liangyuwei/sign_language_robot_ws/test_imi_data/mocap_ik_results_YuMi_g2o_similarity.h5';

pass_points_new = h5read(filename2, ['/', groupname, '/passpoint_traj_1']);


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


