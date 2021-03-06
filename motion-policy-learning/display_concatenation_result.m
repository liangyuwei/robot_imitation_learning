%% Read joint trajectories --> Convert to eef's cartesian pose --> display dynamically

%% Read .h5 file
file_name = 'dual_ur5_whole_traj_concat_2.h5';
% info = h5info(file_name);
joint_traj_l = h5read(file_name, '/traj_pair_approach2_insert2_tmp/joint_traj_l');
joint_traj_r = h5read(file_name, '/traj_pair_approach2_insert2_tmp/joint_traj_r');
joint_timestamp_l = h5read(file_name, '/traj_pair_approach2_insert2_tmp/joint_timestamp_l');
joint_timestamp_r = h5read(file_name, '/traj_pair_approach2_insert2_tmp/joint_timestamp_r');
joint_traj_l_pos = joint_traj_l'; joint_traj_l_pos = joint_traj_l_pos(:, 1:6);
joint_traj_r_pos = joint_traj_r'; joint_traj_r_pos = joint_traj_r_pos(:, 1:6);

%% Joint trajectory --> eef trajectory
[~, wrist_traj_points_l] = obtain_robot_traj(joint_traj_l_pos, true);
[~, wrist_traj_points_r] = obtain_robot_traj(joint_traj_r_pos, false);

%% Alternatives, read eef trajectories directly
cart_traj_filename = 'dual_ur5_whole_cart_traj_concat_2_with_delay.h5';
% info = h5info(cart_traj_filename);
cart_traj_l = h5read(cart_traj_filename, '/traj_pair_approach2_insert2_tmp/cart_traj_l');
cart_traj_r = h5read(cart_traj_filename, '/traj_pair_approach2_insert2_tmp/cart_traj_r');
wrist_traj_points_l = cart_traj_l';
wrist_traj_points_r = cart_traj_r';

% display the path
figure;
plot3(wrist_traj_points_l(:, 1), wrist_traj_points_l(:, 2), wrist_traj_points_l(:, 3), 'b.'); grid on; hold on;
plot3(wrist_traj_points_r(:, 1), wrist_traj_points_r(:, 2), wrist_traj_points_r(:, 3), 'r.');
title('The end-effector''s trajectories');
xlabel('x'); ylabel('y'); zlabel('z');

%% Display dynamically
% display the change of local frame
record = false;
gif_name = 'whole_traj_concat_2.gif'; %'joining_DMP_with_constraint_checking.gif';
display_frame_change(wrist_traj_points_l', wrist_traj_points_r', record, gif_name);
% ([wrist_traj_points_l(:, 1:3), wrist_traj_points_l(:, 6), wrist_traj_points_l(:, 5), wrist_traj_points_l(:, 4)]', ...
%                      [wrist_traj_points_r(:, 1:3), wrist_traj_points_r(:, 6), wrist_traj_points_r(:, 5), wrist_traj_points_r(:, 4)]', record, gif_name);
% 

