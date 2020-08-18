
clear;
clc;

group_name = 'gun_2';

%% null space 

null_file_name = '../motion-retargeting/test_nullspace_symmetric_YuMi.h5';

l_wrist_pos_nullspace = h5read(null_file_name, ['/', group_name, '/l_wrist_pos_nullspace']);
l_elbow_pos_nullspace = h5read(null_file_name, ['/', group_name, '/l_elbow_pos_nullspace']);
r_wrist_pos_nullspace = h5read(null_file_name, ['/', group_name, '/r_wrist_pos_nullspace']);
r_elbow_pos_nullspace = h5read(null_file_name, ['/', group_name, '/r_elbow_pos_nullspace']);


%% for my optimization

num_datapoints = 50;

kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0;
dt = 0.04;


file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';


Mu_lrw = h5read(ori_file_name, ['/', group_name, '/Mu_lrw']);
Mu_lew = h5read(ori_file_name, ['/', group_name, '/Mu_lew']);
Mu_rew = h5read(ori_file_name, ['/', group_name, '/Mu_rew']);
Mu_rw = h5read(ori_file_name, ['/', group_name, '/Mu_rw']);

Sigma_lrw = h5read(ori_file_name, ['/', group_name, '/Sigma_lrw']);
Sigma_lew = h5read(ori_file_name, ['/', group_name, '/Sigma_lew']);
Sigma_rew = h5read(ori_file_name, ['/', group_name, '/Sigma_rew']);
Sigma_rw = h5read(ori_file_name, ['/', group_name, '/Sigma_rw']);

Weights_lrw = h5read(ori_file_name, ['/', group_name, '/Weights_lrw']);
Weights_lew = h5read(ori_file_name, ['/', group_name, '/Weights_lew']);
Weights_rew = h5read(ori_file_name, ['/', group_name, '/Weights_rew']);
Weights_rw = h5read(ori_file_name, ['/', group_name, '/Weights_rw']);


dmp_starts_goals_initial = h5read(file_name, ['/', group_name, '/dmp_starts_goals_initial']);

new_goal_lrw_initial = dmp_starts_goals_initial(1:3); new_start_lrw_initial = dmp_starts_goals_initial(4:6);
new_goal_lew_initial = dmp_starts_goals_initial(7:9); new_start_lew_initial = dmp_starts_goals_initial(10:12);
new_goal_rew_initial = dmp_starts_goals_initial(13:15); new_start_rew_initial = dmp_starts_goals_initial(16:18);
new_goal_rw_initial = dmp_starts_goals_initial(19:21); new_start_rw_initial = dmp_starts_goals_initial(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_initial, new_start_lrw_initial, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_initial, new_start_lew_initial, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_initial, new_start_rew_initial, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_initial, new_start_rw_initial, false);

y_r_wrist_initial = y_rw;
y_l_wrist_initial = y_rw + y_lrw;
y_r_elbow_initial = y_rw + y_rew;
y_l_elbow_initial = y_l_wrist_initial + y_lew;


%% plot
figure;

p1 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r-'); hold on; grid on;
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r-'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r-'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r-');  % initial DMP trajs

p2 = plot3(l_wrist_pos_nullspace(1, :), l_wrist_pos_nullspace(2, :), l_wrist_pos_nullspace(3, :), 'g--'); 
plot3(r_wrist_pos_nullspace(1, :), r_wrist_pos_nullspace(2, :), r_wrist_pos_nullspace(3, :), 'g--'); 
plot3(l_elbow_pos_nullspace(1, :), l_elbow_pos_nullspace(2, :), l_elbow_pos_nullspace(3, :), 'g--'); 
plot3(r_elbow_pos_nullspace(1, :), r_elbow_pos_nullspace(2, :), r_elbow_pos_nullspace(3, :), 'g--');  % initial DMP trajs


