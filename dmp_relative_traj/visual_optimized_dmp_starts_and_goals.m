%% visualize the optimized dmp trajectories and plot the costs history

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
group_name = 'fengren_1';

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

%% Load optimized DMP starts and goals and reproduce relative position trajectories using the optimized starts and goals 
dmp_starts_goals = h5read(file_name, ['/', group_name, '/dmp_starts_goals_1']);
new_goal_lrw = dmp_starts_goals(1:3);
new_start_lrw = dmp_starts_goals(4:6);
new_goal_lew = dmp_starts_goals(7:9);
new_start_lew = dmp_starts_goals(10:12);
new_goal_rew = dmp_starts_goals(13:15);
new_start_rew = dmp_starts_goals(16:18);
new_goal_rw = dmp_starts_goals(19:21);
new_start_rw = dmp_starts_goals(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);


%
figure; plot3(y_rw(1,:), y_rw(2,:), y_rw(3,:), 'r-.'); hold on; grid on;
plot3(r_wrist_pos(1,:), r_wrist_pos(2,:), r_wrist_pos(3,:), 'b-.');
title('r\_wist\_pos');

figure; plot3(y_lrw(1,:), y_lrw(2,:), y_lrw(3,:), 'r-.'); hold on; grid on;
plot3(lr_wrist_pos(1,:), lr_wrist_pos(2,:), lr_wrist_pos(3,:), 'b-.');
title('lr\_wrist\_pos');

figure; plot3(y_lew(1,:), y_lew(2,:), y_lew(3,:), 'r-.'); hold on; grid on;
plot3(l_elbow_wrist_pos(1,:), l_elbow_wrist_pos(2,:), l_elbow_wrist_pos(3,:), 'b-.');
title('l\_elbow\_wrist\_pos');

figure; plot3(y_rew(1,:), y_rew(2,:), y_rew(3,:), 'r-.'); hold on; grid on;
plot3(r_elbow_wrist_pos(1,:), r_elbow_wrist_pos(2,:), r_elbow_wrist_pos(3,:), 'b-.');
title('r\_elbow\_wrist\_pos');
%}


%% Obtain trajectories for body parts
y_r_wrist = y_rw;
y_l_wrist = y_rw + y_lrw;
y_r_elbow = y_rw + y_rew;
y_l_elbow = y_l_wrist + y_lew;

figure;
plot3(l_wrist_pos(1, :), l_wrist_pos(2, :), l_wrist_pos(3, :), 'b--'); hold on; grid on;
plot3(r_wrist_pos(1, :), r_wrist_pos(2, :), r_wrist_pos(3, :), 'b--'); 
plot3(l_elbow_pos(1, :), l_elbow_pos(2, :), l_elbow_pos(3, :), 'b--'); 
plot3(r_elbow_pos(1, :), r_elbow_pos(2, :), r_elbow_pos(3, :), 'b--'); 

plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--');
plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--'); 

view(-45, 45);
% title('Original and reproduced trajectories');
title('Original and generalized trajectories');
xlabel('x'); ylabel('y'); zlabel('z'); 



%% Plot the cost history
per_iteration = 5;
wrist_pos_cost_history = h5read(file_name, ['/', group_name, '/wrist_pos_cost_history']);
wrist_ori_cost_history = h5read(file_name, ['/', group_name, '/wrist_ori_cost_history']);
elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/elbow_pos_cost_history']);
finger_cost_history = h5read(file_name, ['/', group_name, '/finger_cost_history']);
similarity_cost_history = h5read(file_name, ['/', group_name, '/similarity_cost_history']);
smoothness_cost_history = h5read(file_name, ['/', group_name, '/smoothness_cost_history']);
col_cost_history = h5read(file_name, ['/', group_name, '/col_cost_history']);
pos_limit_cost_history = h5read(file_name, ['/', group_name, '/pos_limit_cost_history']);
sim_jacobian_history = h5read(file_name, ['/', group_name, '/sim_jacobian_history']);
track_jacobian_history = h5read(file_name, ['/', group_name, '/track_jacobian_history']);

figure;
plot((1:size(col_cost_history, 2))*per_iteration, sum(col_cost_history), 'b-'); hold on; grid on;
plot((1:size(col_cost_history, 2))*per_iteration, sum(col_cost_history), 'bo');
title('History of collision cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(smoothness_cost_history, 2))*per_iteration, sum(smoothness_cost_history), 'b-'); hold on; grid on;
plot((1:size(smoothness_cost_history, 2))*per_iteration, sum(smoothness_cost_history), 'bo');
title('History of smoothness cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(wrist_pos_cost_history, 2))*per_iteration, sum(wrist_pos_cost_history), 'b-'); hold on; grid on;
plot((1:size(wrist_pos_cost_history, 2))*per_iteration, sum(wrist_pos_cost_history), 'bo');
title('History of wrist position cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(wrist_ori_cost_history, 2))*per_iteration, sum(wrist_ori_cost_history), 'b-'); hold on; grid on;
plot((1:size(wrist_ori_cost_history, 2))*per_iteration, sum(wrist_ori_cost_history), 'bo');
title('History of wrist orientation cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(elbow_pos_cost_history, 2))*per_iteration, sum(elbow_pos_cost_history), 'b-'); hold on; grid on;
plot((1:size(elbow_pos_cost_history, 2))*per_iteration, sum(elbow_pos_cost_history), 'bo');
title('History of elbow position cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(finger_cost_history, 2))*per_iteration, sum(finger_cost_history), 'b-'); hold on; grid on;
plot((1:size(finger_cost_history, 2))*per_iteration, sum(finger_cost_history), 'bo');
title('History of finger angle cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(similarity_cost_history, 2))*per_iteration, similarity_cost_history, 'b-'); hold on; grid on;
plot((1:size(finger_cost_history, 2))*per_iteration, similarity_cost_history, 'bo');
title('History of similarity cost');
xlabel('Iterations'); ylabel('Cost Value'); 

% pre-processing before displaying jacobian
dmp_starts_goals_dof = size(sim_jacobian_history, 1);
num_records = size(sim_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(sim_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of SimilarityConstraint Jacobians');
xlabel('Iterations'); ylabel('Gradients magnitude'); 

num_records = size(track_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(track_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of TrackingConstraint Jacobians');
xlabel('Iterations'); ylabel('Gradients magnitude'); 


%% The change of DMP starts and goals
dmp_starts_goals_original = [lr_wrist_pos(:, end); lr_wrist_pos(:, 1);
                             l_elbow_wrist_pos(:, end); l_elbow_wrist_pos(:, 1);
                             r_elbow_wrist_pos(:, end); r_elbow_wrist_pos(:, 1);
                             r_wrist_pos(:, end); r_wrist_pos(:, 1)];
dmp_starts_goals_final = h5read(file_name, ['/', group_name, '/dmp_starts_goals_1']);
dmp_starts_goals_moved = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved']);
dmp_starts_goals_moved_optimed = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_optimed']);
dmp_starts_goals_moved_pulled = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_pulled']);


%% Analyze condition number of J'*J
cond_nums = [];
for i = 1 : size(track_jacobian_history, 2)
    J = track_jacobian_history(:, i); % column vector
    cond_nums = [cond_nums, cond(J*J')];
end
disp(['History of condition numbers of J^T*J = ', num2str(cond_nums)]);


% History of condition numbers of J^T*J = 7.388521923723018e+20  2.343177021191475e+20  3.929846188521127e+20  2.947066452778979e+20  3.816431692540564e+20  2.745309753057258e+22





