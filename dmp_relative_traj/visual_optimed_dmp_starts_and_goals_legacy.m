%% visual_optimed_dmp_starts_and_goals_legacy


clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity-preiter-backuph5';
group_name = 'fengren_1';%'kai_2';%'baozhu_1';%'fengren_1';

num_datapoints = 50;

kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0;
dt = 0.04;


%% Prepare the original data for comparison
time = h5read(ori_file_name, ['/', group_name, '/time']);
l_wrist_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/l_wrist_pos']), num_datapoints, false);
l_elbow_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/l_elbow_pos']), num_datapoints, false);
r_wrist_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/r_wrist_pos']), num_datapoints, false);
r_elbow_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/r_elbow_pos']), num_datapoints, false); % input should be DOF x length

lr_wrist_pos = l_wrist_pos - r_wrist_pos;
l_elbow_wrist_pos = l_elbow_pos - l_wrist_pos;
r_elbow_wrist_pos = r_elbow_pos - r_wrist_pos; 

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

%% Load and prepare data
% 1 - Initial DMP trajs
dmp_starts_goals_initial = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved']);

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


% 2 - preiteration result
preiter_l_wrist_pos_traj = h5read(file_name, ['/', group_name, '/preiter_l_wrist_pos_traj']);
preiter_r_wrist_pos_traj = h5read(file_name, ['/', group_name, '/preiter_r_wrist_pos_traj']);
preiter_l_elbow_pos_traj = h5read(file_name, ['/', group_name, '/preiter_l_elbow_pos_traj']);
preiter_r_elbow_pos_traj = h5read(file_name, ['/', group_name, '/preiter_r_elbow_pos_traj']);


% 3 - Moved Pulled (to actually executed trajs; after pre-iteration)
dmp_starts_goals_moved_pulled = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_pulled']);

new_goal_lrw_moved_pulled = dmp_starts_goals_moved_pulled(1:3); new_start_lrw_moved_pulled = dmp_starts_goals_moved_pulled(4:6);
new_goal_lew_moved_pulled = dmp_starts_goals_moved_pulled(7:9); new_start_lew_moved_pulled = dmp_starts_goals_moved_pulled(10:12);
new_goal_rew_moved_pulled = dmp_starts_goals_moved_pulled(13:15); new_start_rew_moved_pulled = dmp_starts_goals_moved_pulled(16:18);
new_goal_rw_moved_pulled = dmp_starts_goals_moved_pulled(19:21); new_start_rw_moved_pulled = dmp_starts_goals_moved_pulled(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_moved_pulled, new_start_lrw_moved_pulled, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_moved_pulled, new_start_lew_moved_pulled, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_moved_pulled, new_start_rew_moved_pulled, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_moved_pulled, new_start_rw_moved_pulled, false);

y_r_wrist_moved_pulled = y_rw;
y_l_wrist_moved_pulled = y_rw + y_lrw;
y_r_elbow_moved_pulled = y_rw + y_rew;
y_l_elbow_moved_pulled = y_l_wrist_moved_pulled + y_lew;


% 4 - Moved Optimed (this is performed before everything else... trying to move DMP from original imitated localtion to robots by optimization)
dmp_starts_goals_moved_optimed = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_pulled']);

new_goal_lrw_moved_optimed = dmp_starts_goals_moved_optimed(1:3); new_start_lrw_moved_optimed = dmp_starts_goals_moved_optimed(4:6);
new_goal_lew_moved_optimed = dmp_starts_goals_moved_optimed(7:9); new_start_lew_moved_optimed = dmp_starts_goals_moved_optimed(10:12);
new_goal_rew_moved_optimed = dmp_starts_goals_moved_optimed(13:15); new_start_rew_moved_optimed = dmp_starts_goals_moved_optimed(16:18);
new_goal_rw_moved_optimed = dmp_starts_goals_moved_optimed(19:21); new_start_rw_moved_optimed = dmp_starts_goals_moved_optimed(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_moved_optimed, new_start_lrw_moved_optimed, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_moved_optimed, new_start_lew_moved_optimed, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_moved_optimed, new_start_rew_moved_optimed, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_moved_optimed, new_start_rw_moved_optimed, false);

y_r_wrist_moved_optimed = y_rw;
y_l_wrist_moved_optimed = y_rw + y_lrw;
y_r_elbow_moved_optimed = y_rw + y_rew;
y_l_elbow_moved_optimed = y_l_wrist_moved_optimed + y_lew;


%% Plot comparison results
% Fig 1 - Comparison between human demonstrated trajs, initial DMP trajs and pre-iteration results (with starts and goals manually set)    
figure;
p1 = plot3(l_wrist_pos(1, :), l_wrist_pos(2, :), l_wrist_pos(3, :), 'b--'); hold on; grid on;
plot3(r_wrist_pos(1, :), r_wrist_pos(2, :), r_wrist_pos(3, :), 'b--'); 
plot3(l_elbow_pos(1, :), l_elbow_pos(2, :), l_elbow_pos(3, :), 'b--'); 
plot3(r_elbow_pos(1, :), r_elbow_pos(2, :), r_elbow_pos(3, :), 'b--');  % original imitation data

p2 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); 
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');  % initial DMP trajs (moved manually)

p3 = plot3(preiter_l_wrist_pos_traj(1, :), preiter_l_wrist_pos_traj(2, :), preiter_l_wrist_pos_traj(3, :), 'g--');
plot3(preiter_r_wrist_pos_traj(1, :), preiter_r_wrist_pos_traj(2, :), preiter_r_wrist_pos_traj(3, :), 'g--'); 
plot3(preiter_l_elbow_pos_traj(1, :), preiter_l_elbow_pos_traj(2, :), preiter_l_elbow_pos_traj(3, :), 'g--'); 
plot3(preiter_r_elbow_pos_traj(1, :), preiter_r_elbow_pos_traj(2, :), preiter_r_elbow_pos_traj(3, :), 'g--');  % pre-iteration (tracked, actually executed trajs)

view(-45, 45);
title('Human demonstrated trajs, Initial DMP trajs and Tracked trajs', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 
legend([p1(1), p2(1), p3(1)], 'Demonstrated trajs', 'Initial DMP trajs', 'Tracked trajs', 'Location', 'NorthEastOutside');


% Fig 2 - Close up
figure;

p2 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');  % initial DMP trajs (moved manually)

p3 = plot3(preiter_l_wrist_pos_traj(1, :), preiter_l_wrist_pos_traj(2, :), preiter_l_wrist_pos_traj(3, :), 'g--');
plot3(preiter_r_wrist_pos_traj(1, :), preiter_r_wrist_pos_traj(2, :), preiter_r_wrist_pos_traj(3, :), 'g--'); 
plot3(preiter_l_elbow_pos_traj(1, :), preiter_l_elbow_pos_traj(2, :), preiter_l_elbow_pos_traj(3, :), 'g--'); 
plot3(preiter_r_elbow_pos_traj(1, :), preiter_r_elbow_pos_traj(2, :), preiter_r_elbow_pos_traj(3, :), 'g--');  % pre-iteration (tracked, actually executed trajs)

view(-45, 45);
title('Initial DMP trajs and Tracked trajs', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 
legend([p2(1), p3(1)], 'Initial DMP trajs', 'Tracked trajs', 'Location', 'NorthEastOutside');


% Fig 3 - Initial DMP trajs, Tracked trajs and Pulled trajs (pulled to actually executed trajs)
figure;

p1 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');  % initial DMP trajs (moved manually)

p2 = plot3(preiter_l_wrist_pos_traj(1, :), preiter_l_wrist_pos_traj(2, :), preiter_l_wrist_pos_traj(3, :), 'g--');
plot3(preiter_r_wrist_pos_traj(1, :), preiter_r_wrist_pos_traj(2, :), preiter_r_wrist_pos_traj(3, :), 'g--'); 
plot3(preiter_l_elbow_pos_traj(1, :), preiter_l_elbow_pos_traj(2, :), preiter_l_elbow_pos_traj(3, :), 'g--'); 
plot3(preiter_r_elbow_pos_traj(1, :), preiter_r_elbow_pos_traj(2, :), preiter_r_elbow_pos_traj(3, :), 'g--');  % pre-iteration (tracked, actually executed trajs)

p3 = plot3(y_l_wrist_moved_pulled(1, :), y_l_wrist_moved_pulled(2, :), y_l_wrist_moved_pulled(3, :), 'b--'); 
plot3(y_r_wrist_moved_pulled(1, :), y_r_wrist_moved_pulled(2, :), y_r_wrist_moved_pulled(3, :), 'b--'); 
plot3(y_l_elbow_moved_pulled(1, :), y_l_elbow_moved_pulled(2, :), y_l_elbow_moved_pulled(3, :), 'b--'); 
plot3(y_r_elbow_moved_pulled(1, :), y_r_elbow_moved_pulled(2, :), y_r_elbow_moved_pulled(3, :), 'b--');  % moved_pulled DMP trajs (moved manually)

view(-45, 45);
title('Initial DMP trajs, Tracked trajs and Pulled trajs', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 
legend([p1(1), p2(1), p3(1)], 'Initial DMP trajs', 'Tracked trajs', 'Pulled trajs', 'Location', 'NorthEastOutside');


