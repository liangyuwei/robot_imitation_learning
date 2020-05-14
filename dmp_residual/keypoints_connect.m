function y_new = keypoints_connect(tmp_mat_file, pass_points_new)
%% This function reads a list of pass_points(including via-points, start and goal).


%% Preparation
load(tmp_mat_file);

%% Adapt to new via-points
% model the piece-wise linear function
ele_traj_struct_new = elementary_trajectory_model(pass_points_new, pass_time, pos_and_glove_id, quat_id);

% compute h(x) sequence
h_new = elementary_trajectory_calculate_sequence(ele_traj_struct_new, time_range, pos_and_glove_id, quat_id);

% get y(x)
y_new = get_final(h_new, f_seq, pos_and_glove_id, quat_id);


end


