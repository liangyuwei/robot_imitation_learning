%% visualize the optimized dmp trajectories and plot the costs history

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
group_name = 'baozhu_1';

num_datapoints = 50;

kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0;
dt = 0.04;

max_round = h5read(file_name, ['/', group_name, '/max_round']); % for ease of display

title_font_size = 18; %16;
label_font_size = 14; %16;


%% Prepare the original data for comparison
time = h5read(ori_file_name, ['/', group_name, '/time']);
l_wrist_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/l_wrist_pos_adjusted']), num_datapoints, false);
l_elbow_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/l_elbow_pos_adjusted']), num_datapoints, false);
r_wrist_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/r_wrist_pos_adjusted']), num_datapoints, false);
r_elbow_pos = resample_traj(time, h5read(ori_file_name, ['/', group_name, '/r_elbow_pos_adjusted']), num_datapoints, false); % input should be DOF x length

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
%{
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

% plot relative trajs and rw
%{
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

% Obtain trajectories for body parts
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
%}


%% Plot the cost history
per_iteration = 20;%1;
wrist_pos_cost_history = h5read(file_name, ['/', group_name, '/wrist_pos_cost_history_q_optim']);
l_wrist_pos_cost_history = h5read(file_name, ['/', group_name, '/l_wrist_pos_cost_history_q_optim']);
r_wrist_pos_cost_history = h5read(file_name, ['/', group_name, '/r_wrist_pos_cost_history_q_optim']);
wrist_ori_cost_history = h5read(file_name, ['/', group_name, '/wrist_ori_cost_history_q_optim']);
elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/elbow_pos_cost_history_q_optim']);
l_elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/l_elbow_pos_cost_history_q_optim']);
r_elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/r_elbow_pos_cost_history_q_optim']);
finger_cost_history = h5read(file_name, ['/', group_name, '/finger_cost_history']);
l_finger_cost_history = h5read(file_name, ['/', group_name, '/l_finger_cost_history']);
r_finger_cost_history = h5read(file_name, ['/', group_name, '/r_finger_cost_history']);
% similarity_cost_history = h5read(file_name, ['/', group_name, '/similarity_cost_history']);
smoothness_cost_history = h5read(file_name, ['/', group_name, '/smoothness_cost_history']);
col_cost_history = h5read(file_name, ['/', group_name, '/col_cost_history']);
% pos_limit_cost_history = h5read(file_name, ['/', group_name, '/pos_limit_cost_history']);
dmp_scale_cost_history = h5read(file_name, ['/', group_name, '/dmp_scale_cost_history']);
dmp_orien_cost_history = h5read(file_name, ['/', group_name, '/dmp_orien_cost_history']);
dmp_rel_change_cost_history = h5read(file_name, ['/', group_name, '/dmp_rel_change_cost_history']);

% jacobians for DMP starts and goals
% sim_jacobian_history = h5read(file_name, ['/', group_name, '/sim_jacobian_history']);
% track_jacobian_history = h5read(file_name, ['/', group_name, '/track_jacobian_history']);
orien_jacobian_history = h5read(file_name, ['/', group_name, '/orien_jacobian_history']);
scale_jacobian_history = h5read(file_name, ['/', group_name, '/scale_jacobian_history']);

% update history of DMP
dmp_update_history = h5read(file_name, ['/', group_name, '/dmp_update_history']);

% plot all
figure;
plot((1:size(col_cost_history, 2))*per_iteration, sum(col_cost_history), 'b-'); hold on; grid on;
% plot((1:size(col_cost_history, 2))*per_iteration, sum(col_cost_history), 'bo');
title('History of collision cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

% figure;
% plot((1:size(pos_limit_cost_history, 2))*per_iteration, sum(pos_limit_cost_history), 'b-'); hold on; grid on;
% % plot((1:size(pos_limit_cost_history, 2))*per_iteration, sum(pos_limit_cost_history), 'bo');
% title('History of position limit cost', 'FontSize', 18);
% xlabel('Iterations', 'FontSize', 18); ylabel('Cost Value', 'FontSize', 18); 

figure;
plot((1:size(smoothness_cost_history, 2))*per_iteration, sum(smoothness_cost_history), 'b-'); hold on; grid on;
% plot((1:size(smoothness_cost_history, 2))*per_iteration, sum(smoothness_cost_history), 'bo');
title('History of smoothness cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(wrist_pos_cost_history, 2))*per_iteration, sum(wrist_pos_cost_history), 'b-'); hold on; grid on;
% plot((1:size(wrist_pos_cost_history, 2))*per_iteration, sum(wrist_pos_cost_history), 'bo');
title('History of wrist position cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(l_wrist_pos_cost_history, 2))*per_iteration, sum(l_wrist_pos_cost_history), 'b-'); hold on; grid on;
% plot((1:size(l_wrist_pos_cost_history, 2))*per_iteration, sum(l_wrist_pos_cost_history), 'bo');
title('History of left wrist position cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(r_wrist_pos_cost_history, 2))*per_iteration, sum(r_wrist_pos_cost_history), 'b-'); hold on; grid on;
% plot((1:size(r_wrist_pos_cost_history, 2))*per_iteration, sum(r_wrist_pos_cost_history), 'bo');
title('History of right wrist position cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(wrist_ori_cost_history, 2))*per_iteration, sum(wrist_ori_cost_history), 'b-'); hold on; grid on;
% plot((1:size(wrist_ori_cost_history, 2))*per_iteration, sum(wrist_ori_cost_history), 'bo');
title('History of wrist orientation cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(elbow_pos_cost_history, 2))*per_iteration, sum(elbow_pos_cost_history), 'b-'); hold on; grid on;
% plot((1:size(elbow_pos_cost_history, 2))*per_iteration, sum(elbow_pos_cost_history), 'bo');
title('History of elbow position cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(finger_cost_history, 2))*per_iteration, sum(finger_cost_history), 'b-'); hold on; grid on;
% plot((1:size(finger_cost_history, 2))*per_iteration, sum(finger_cost_history), 'bo');
title('History of finger angle cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(l_finger_cost_history, 2))*per_iteration, sum(l_finger_cost_history), 'b-'); hold on; grid on;
% plot((1:size(finger_cost_history, 2))*per_iteration, sum(finger_cost_history), 'bo');
title('History of left finger angle cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(r_finger_cost_history, 2))*per_iteration, sum(r_finger_cost_history), 'b-'); hold on; grid on;
% plot((1:size(finger_cost_history, 2))*per_iteration, sum(finger_cost_history), 'bo');
title('History of right finger angle cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(dmp_orien_cost_history, 2))*per_iteration, dmp_orien_cost_history, 'b-'); hold on; grid on;
% plot((1:size(dmp_orien_cost_history, 2))*per_iteration, dmp_orien_cost_history, 'bo');
title('History of dmp orientation cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(dmp_scale_cost_history, 2))*per_iteration, dmp_scale_cost_history, 'b-'); hold on; grid on;
% plot((1:size(dmp_scale_cost_history, 2))*per_iteration, dmp_scale_cost_history, 'bo');
title('History of dmp scale cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

figure;
plot((1:size(dmp_rel_change_cost_history, 2))*per_iteration, dmp_rel_change_cost_history, 'b-'); hold on; grid on;
% plot((1:size(dmp_rel_change_cost_history, 2))*per_iteration, dmp_rel_change_cost_history, 'bo');
title('History of dmp rel change cost', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Cost Value', 'FontSize', label_font_size); 

% figure;
% plot((1:size(similarity_cost_history, 2))*per_iteration, similarity_cost_history, 'b-'); hold on; grid on;
% % plot((1:size(similarity_cost_history, 2))*per_iteration, similarity_cost_history, 'bo');
% title('History of similarity cost', 'FontSize', 18);
% xlabel('Iterations', 'FontSize', 18); ylabel('Cost Value', 'FontSize', 18); 


%% Plot jacobian history
%{
% pre-processing before displaying jacobian
% dmp_starts_goals_dof = size(sim_jacobian_history, 1);
% num_records = size(sim_jacobian_history, 2);
% jacobian_norm = zeros(1, num_records);
% for n = 1 : num_records
%     jacobian_norm(n) = norm(sim_jacobian_history(:, n));
% end
% figure;
% plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
% % plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
% title('History of SimilarityConstraint Jacobians', 'FontSize', 18);
% xlabel('Iterations', 'FontSize', 18); ylabel('Gradients magnitude', 'FontSize', 18); 

num_records = size(track_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(track_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
% plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of TrackingConstraint Jacobians', 'FontSize', 18);
xlabel('Iterations', 'FontSize', 18); ylabel('Gradients magnitude', 'FontSize', 18); 

num_records = size(orien_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(orien_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
% plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of DMPOrienConstraint Jacobians', 'FontSize', 18);
xlabel('Iterations', 'FontSize', 18); ylabel('Gradients magnitude', 'FontSize', 18); 

num_records = size(scale_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(scale_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
% plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of DMPScaleConstraint Jacobians', 'FontSize', 18);
xlabel('Iterations', 'FontSize', 18); ylabel('Gradients magnitude', 'FontSize', 18); 

% should use norm() instead of sum(), which is more reasonable
% not exactly the change history, since it is recorded after convergence!!!
num_records = size(dmp_update_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(dmp_update_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
% plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of DMP updates', 'FontSize', 18);
xlabel('Iterations', 'FontSize', 18); ylabel('Norm of updates on all DOF', 'FontSize', 18); 

%}


%% The change of DMP starts and goals
dmp_starts_goals_original = [lr_wrist_pos(:, end); lr_wrist_pos(:, 1);
                             l_elbow_wrist_pos(:, end); l_elbow_wrist_pos(:, 1);
                             r_elbow_wrist_pos(:, end); r_elbow_wrist_pos(:, 1);
                             r_wrist_pos(:, end); r_wrist_pos(:, 1)];
dmp_starts_goals_final = h5read(file_name, ['/', group_name, '/dmp_starts_goals_optimed_', num2str(max_round)]);
% dmp_starts_goals_moved = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved']);
% dmp_starts_goals_moved_optimed = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_optimed']);
% dmp_starts_goals_moved_pulled = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_pulled']);


%% Analyze condition number of J'*J
%{
cond_nums = [];
for i = 1 : size(track_jacobian_history, 2)
    J = track_jacobian_history(:, i); % column vector
    cond_nums = [cond_nums, cond(J*J')];
end
disp(['History of condition numbers of J^T*J = ', num2str(cond_nums)]);

% History of condition numbers of J^T*J = 7.388521923723018e+20  2.343177021191475e+20  3.929846188521127e+20  2.947066452778979e+20  3.816431692540564e+20  2.745309753057258e+22
%}


%% Constraints related to DMP starts and goals
% get goals and starts
lrw_goal_ori = dmp_starts_goals_original(1:3);
lrw_start_ori = dmp_starts_goals_original(4:6);
lew_goal_ori = dmp_starts_goals_original(7:9);
lew_start_ori = dmp_starts_goals_original(10:12);
rew_goal_ori = dmp_starts_goals_original(13:15);
rew_start_ori = dmp_starts_goals_original(16:18);
rw_goal_ori = dmp_starts_goals_original(19:21);
rw_start_ori = dmp_starts_goals_original(22:24);

lrw_goal_new = dmp_starts_goals_final(1:3);
lrw_start_new = dmp_starts_goals_final(4:6);
lew_goal_new = dmp_starts_goals_final(7:9);
lew_start_new = dmp_starts_goals_final(10:12);
rew_goal_new = dmp_starts_goals_final(13:15);
rew_start_new = dmp_starts_goals_final(16:18);
rw_goal_new = dmp_starts_goals_final(19:21);
rw_start_new = dmp_starts_goals_final(22:24);

% get vectors pointing from starts to goals
lrw_vec_ori = lrw_goal_ori - lrw_start_ori;
lew_vec_ori = lew_goal_ori - lew_start_ori;
rew_vec_ori = rew_goal_ori - rew_start_ori;
rw_vec_ori = rw_goal_ori - rw_start_ori;

lrw_vec_new = lrw_goal_new - lrw_start_new;
lew_vec_new = lew_goal_new - lew_start_new;
rew_vec_new = rew_goal_new - rew_start_new;
rw_vec_new = rw_goal_new - rw_start_new;

% constraints-related values
lrw_ratio = norm(lrw_vec_new) / norm(lrw_vec_ori)
lew_ratio = norm(lew_vec_new) / norm(lew_vec_ori)
rew_ratio = norm(rew_vec_new) / norm(rew_vec_ori)
rw_ratio = norm(rw_vec_new) / norm(rw_vec_ori)

lrw_th = acos(lrw_vec_new'*lrw_vec_ori/norm(lrw_vec_new)/norm(lrw_vec_ori)) * 180 / pi
lew_th = acos(lew_vec_new'*lew_vec_ori/norm(lew_vec_new)/norm(lew_vec_ori)) * 180 / pi
rew_th = acos(rew_vec_new'*rew_vec_ori/norm(rew_vec_new)/norm(rew_vec_ori)) * 180 / pi
rw_th = acos(rw_vec_new'*rw_vec_ori/norm(rw_vec_new)/norm(rw_vec_ori)) * 180 / pi % in degree !!

% Evaluate magnitudes of changes of relative DMPs
lrw_goal_change = norm(lrw_goal_ori - lrw_goal_new)
lrw_start_change = norm(lrw_start_ori - lrw_start_new)

lew_goal_change = norm(lew_goal_ori - lew_goal_new)
lew_start_change = norm(lew_start_ori - lew_start_new)

rew_goal_change = norm(rew_goal_ori - rew_goal_new)
rew_start_change = norm(rew_start_ori - rew_start_new)

rw_goal_change = norm(rw_goal_ori - rw_goal_new)
rw_start_change = norm(rw_start_ori - rw_start_new)


%% Debug: analyze jacobian history
% raw data, 24-dim
% figure;
% p1 = plot((1:size(orien_jacobian_history, 2))*per_iteration, orien_jacobian_history, 'b-'); hold on; grid on;
% plot((1:size(orien_jacobian_history, 2))*per_iteration, orien_jacobian_history, 'bo'); % 1
% p2 = plot((1:size(scale_jacobian_history, 2))*per_iteration, scale_jacobian_history, 'r-'); 
% % plot((1:size(scale_jacobian_history, 2))*per_iteration, scale_jacobian_history, 'ro'); % 2
% p3 = plot((1:size(track_jacobian_history, 2))*per_iteration, track_jacobian_history, 'g-'); 
% % plot((1:size(track_jacobian_history, 2))*per_iteration, track_jacobian_history, 'go'); % 3
% % p4 = plot((1:size(sim_jacobian_history, 2))*per_iteration, sim_jacobian_history, 'm-'); 
% % plot((1:size(sim_jacobian_history, 2))*per_iteration, sim_jacobian_history, 'mo'); % 4 
% title('History of Jacobians w.r.t DMP starts and goals', 'FontSize', 18);
% xlabel('Iterations', 'FontSize', 18); ylabel('Jacobians', 'FontSize', 18); 
% legend([p1(1), p2(1), p3(1)], 'orien\_jacobian', 'scale\_jacobian', 'track\_jacobian');%, 'Location', 'NorthEastOutside');
% % 'track_jacobian', 'sim_jacobian', 'Location', 'NorthEastOutside');

% norms
num_records = size(orien_jacobian_history, 2);
% track_jacobian_norm = zeros(1, num_records);
% sim_jacobian_norm = zeros(1, num_records);
orien_jacobian_norm = zeros(1, num_records);
scale_jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
%     track_jacobian_norm(n) = norm(track_jacobian_history(:, n));
%     sim_jacobian_norm(n) = norm(sim_jacobian_history(:, n));
    orien_jacobian_norm(n) = norm(orien_jacobian_history(:, n));
    scale_jacobian_norm(n) = norm(scale_jacobian_history(:, n));
end
figure;
p1 = plot((1:num_records)*per_iteration, orien_jacobian_norm, 'b-'); hold on; grid on;
p2 = plot((1:num_records)*per_iteration, scale_jacobian_norm, 'r-');
% p3 = plot((1:num_records)*per_iteration, track_jacobian_norm, 'g-');
% p4 = plot((1:num_records)*per_iteration, sim_jacobian_norm, 'm-');
title('History of Norms of DMP Jacobians', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('Iterations', 'FontSize', label_font_size); ylabel('Norms of jacobians', 'FontSize', label_font_size); 
legend([p1, p2], 'orien\_jacobian', 'scale\_jacobian', 'Location', 'NorthEastOutside');


%% Optimization results of First Round (q optimization -> manually move DMP starts and goals -> DMP optimization)
% 1 - load initial DMP trajs (the one set manually in the beginning)
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

% 2 - load actual executed trajs after q optimization in the first round
actual_l_wrist_pos_traj_0 = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_0']);
actual_r_wrist_pos_traj_0 = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_0']);
actual_l_elbow_pos_traj_0 = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_0']);
actual_r_elbow_pos_traj_0 = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_0']);

% 2.5 - load actually executed trajs of next round
% actual_l_wrist_pos_traj_1 = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_1']);
% actual_r_wrist_pos_traj_1 = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_1']);
% actual_l_elbow_pos_traj_1 = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_1']);
% actual_r_elbow_pos_traj_1 = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_1']);

% 2.75 - the last round, actually executed trajs
actual_l_wrist_pos_traj_last = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_', num2str(max_round)]);
actual_r_wrist_pos_traj_last = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_', num2str(max_round)]);
actual_l_elbow_pos_traj_last = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_', num2str(max_round)]);
actual_r_elbow_pos_traj_last = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_', num2str(max_round)]);

% 3 - load manually moved DMP trajs
%{
dmp_starts_goals_moved_0 = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_0']);

new_goal_lrw_moved_0 = dmp_starts_goals_moved_0(1:3); new_start_lrw_moved_0 = dmp_starts_goals_moved_0(4:6);
new_goal_lew_moved_0 = dmp_starts_goals_moved_0(7:9); new_start_lew_moved_0 = dmp_starts_goals_moved_0(10:12);
new_goal_rew_moved_0 = dmp_starts_goals_moved_0(13:15); new_start_rew_moved_0 = dmp_starts_goals_moved_0(16:18);
new_goal_rw_moved_0 = dmp_starts_goals_moved_0(19:21); new_start_rw_moved_0 = dmp_starts_goals_moved_0(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_moved_0, new_start_lrw_moved_0, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_moved_0, new_start_lew_moved_0, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_moved_0, new_start_rew_moved_0, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_moved_0, new_start_rw_moved_0, false);

y_r_wrist_moved_0 = y_rw;
y_l_wrist_moved_0 = y_rw + y_lrw;
y_r_elbow_moved_0 = y_rw + y_rew;
y_l_elbow_moved_0 = y_l_wrist_moved_0 + y_lew;

%}

% 4 - load optimized DMP trajs
dmp_starts_goals_optimed_0 = h5read(file_name, ['/', group_name, '/dmp_starts_goals_optimed_0']);

new_goal_lrw_optimed_0 = dmp_starts_goals_optimed_0(1:3); new_start_lrw_optimed_0 = dmp_starts_goals_optimed_0(4:6);
new_goal_lew_optimed_0 = dmp_starts_goals_optimed_0(7:9); new_start_lew_optimed_0 = dmp_starts_goals_optimed_0(10:12);
new_goal_rew_optimed_0 = dmp_starts_goals_optimed_0(13:15); new_start_rew_optimed_0 = dmp_starts_goals_optimed_0(16:18);
new_goal_rw_optimed_0 = dmp_starts_goals_optimed_0(19:21); new_start_rw_optimed_0 = dmp_starts_goals_optimed_0(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_optimed_0, new_start_lrw_optimed_0, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_optimed_0, new_start_lew_optimed_0, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_optimed_0, new_start_rew_optimed_0, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_optimed_0, new_start_rw_optimed_0, false);

y_r_wrist_optimed_0 = y_rw;
y_l_wrist_optimed_0 = y_rw + y_lrw;
y_r_elbow_optimed_0 = y_rw + y_rew;
y_l_elbow_optimed_0 = y_l_wrist_optimed_0 + y_lew;

% 5 - load last-round optimized DMP trajs
dmp_starts_goals_optimed_last = h5read(file_name, ['/', group_name, '/dmp_starts_goals_optimed_', num2str(max_round)]);

new_goal_lrw_optimed_last = dmp_starts_goals_optimed_last(1:3); new_start_lrw_optimed_last = dmp_starts_goals_optimed_last(4:6);
new_goal_lew_optimed_last = dmp_starts_goals_optimed_last(7:9); new_start_lew_optimed_last = dmp_starts_goals_optimed_last(10:12);
new_goal_rew_optimed_last = dmp_starts_goals_optimed_last(13:15); new_start_rew_optimed_last = dmp_starts_goals_optimed_last(16:18);
new_goal_rw_optimed_last = dmp_starts_goals_optimed_last(19:21); new_start_rw_optimed_last = dmp_starts_goals_optimed_last(22:24);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_optimed_last, new_start_lrw_optimed_last, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_optimed_last, new_start_lew_optimed_last, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_optimed_last, new_start_rew_optimed_last, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_optimed_last, new_start_rw_optimed_last, false);

y_r_wrist_optimed_last = y_rw;
y_l_wrist_optimed_last = y_rw + y_lrw;
y_r_elbow_optimed_last = y_rw + y_rew;
y_l_elbow_optimed_last = y_l_wrist_optimed_last + y_lew;

% Fig 1 - Comparison between human demonstrated trajs and initial DMP trajs (with starts and goals manually set)    
figure;
p1 = plot3(l_wrist_pos(1, :), l_wrist_pos(2, :), l_wrist_pos(3, :), 'b--'); hold on; grid on;
plot3(r_wrist_pos(1, :), r_wrist_pos(2, :), r_wrist_pos(3, :), 'b--'); 
plot3(l_elbow_pos(1, :), l_elbow_pos(2, :), l_elbow_pos(3, :), 'b--'); 
plot3(r_elbow_pos(1, :), r_elbow_pos(2, :), r_elbow_pos(3, :), 'b--');  % original imitation data

p2 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');  % initial DMP trajs

view(-45, 45);
title('Human demonstrated trajectories and Initial DMP trajectories', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('x', 'FontSize', label_font_size); ylabel('y', 'FontSize', label_font_size); zlabel('z', 'FontSize', label_font_size); 
legend([p1(1), p2(1)], 'Demonstrated trajs', 'Initial DMP trajs');%, 'Location', 'NorthEastOutside', 'FontSize', 16);


% Fig 2 - Comparison between initial DMP trajs and tracked(actually executed) trajs     
figure;
p1 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');  % initial DMP trajs

p2 = plot3(actual_l_wrist_pos_traj_0(1, :), actual_l_wrist_pos_traj_0(2, :), actual_l_wrist_pos_traj_0(3, :), 'g--');
plot3(actual_r_wrist_pos_traj_0(1, :), actual_r_wrist_pos_traj_0(2, :), actual_r_wrist_pos_traj_0(3, :), 'g--'); 
plot3(actual_l_elbow_pos_traj_0(1, :), actual_l_elbow_pos_traj_0(2, :), actual_l_elbow_pos_traj_0(3, :), 'g--'); 
plot3(actual_r_elbow_pos_traj_0(1, :), actual_r_elbow_pos_traj_0(2, :), actual_r_elbow_pos_traj_0(3, :), 'g--');  % tracked (actually executed) trajs

view(-45, 45);
title('Initial DMP trajectories and Tracked (actually executed) trajectories', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('x', 'FontSize', label_font_size); ylabel('y', 'FontSize', label_font_size); zlabel('z', 'FontSize', label_font_size); 
legend([p1(1), p2(1)], 'Initial DMP trajs', 'Tracked trajs');%, 'Location', 'NorthEastOutside', 'FontSize', 16);


% split view of each dof
% plot_3d_subplots(y_l_wrist_initial, actual_l_wrist_pos_traj_0, 'Left Wrist Pos', 'Initial DMP traj', 'Tracked trajs');
% plot_3d_subplots(y_l_elbow_initial, actual_l_elbow_pos_traj_0, 'Left Elbow Pos', 'Initial DMP traj', 'Tracked trajs');
% plot_3d_subplots(y_r_wrist_initial, actual_r_wrist_pos_traj_0, 'Right Wrist Pos', 'Initial DMP traj', 'Tracked trajs');
% plot_3d_subplots(y_r_elbow_initial, actual_r_elbow_pos_traj_0, 'Right Elbow Pos', 'Initial DMP traj', 'Tracked trajs');


% Fig 3 - Comparison between tracked(actually executed) trajs and moved DMP trajs  
%{
figure;
p1 = plot3(actual_l_wrist_pos_traj_0(1, :), actual_l_wrist_pos_traj_0(2, :), actual_l_wrist_pos_traj_0(3, :), 'g--'); hold on; grid on;
plot3(actual_r_wrist_pos_traj_0(1, :), actual_r_wrist_pos_traj_0(2, :), actual_r_wrist_pos_traj_0(3, :), 'g--'); 
plot3(actual_l_elbow_pos_traj_0(1, :), actual_l_elbow_pos_traj_0(2, :), actual_l_elbow_pos_traj_0(3, :), 'g--'); 
plot3(actual_r_elbow_pos_traj_0(1, :), actual_r_elbow_pos_traj_0(2, :), actual_r_elbow_pos_traj_0(3, :), 'g--');  % tracked (actually executed) trajs

p2 = plot3(y_l_wrist_moved_0(1, :), y_l_wrist_moved_0(2, :), y_l_wrist_moved_0(3, :), 'r-*');
plot3(y_r_wrist_moved_0(1, :), y_r_wrist_moved_0(2, :), y_r_wrist_moved_0(3, :), 'r-*'); 
plot3(y_l_elbow_moved_0(1, :), y_l_elbow_moved_0(2, :), y_l_elbow_moved_0(3, :), 'r-*'); 
plot3(y_r_elbow_moved_0(1, :), y_r_elbow_moved_0(2, :), y_r_elbow_moved_0(3, :), 'r-*'); % moved DMP trajs

p3 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); 
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--'); 
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');  % initial DMP trajs
 
view(-45, 45);
title('Tracked (actually executed) trajs, Moved DMP trajs and Initial DMP trajs', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 
legend([p1(1), p2(1), p3(1)], 'Tracked trajs', 'Moved DMP trajs', 'Initial DMP trajs', 'Location', 'NorthEastOutside', 'FontSize', 16);
%}

% Fig 4 - Comparison between Moved DMP trajs and Optimed DMP trajs
%{
figure;

p1 = plot3(y_l_wrist_moved_0(1, :), y_l_wrist_moved_0(2, :), y_l_wrist_moved_0(3, :), 'r-*'); hold on; grid on;
plot3(y_r_wrist_moved_0(1, :), y_r_wrist_moved_0(2, :), y_r_wrist_moved_0(3, :), 'r-*'); 
plot3(y_l_elbow_moved_0(1, :), y_l_elbow_moved_0(2, :), y_l_elbow_moved_0(3, :), 'r-*'); 
plot3(y_r_elbow_moved_0(1, :), y_r_elbow_moved_0(2, :), y_r_elbow_moved_0(3, :), 'r-*'); % moved DMP trajs

p2 = plot3(y_l_wrist_optimed_0(1, :), y_l_wrist_optimed_0(2, :), y_l_wrist_optimed_0(3, :), 'r--'); 
plot3(y_r_wrist_optimed_0(1, :), y_r_wrist_optimed_0(2, :), y_r_wrist_optimed_0(3, :), 'r--'); 
plot3(y_l_elbow_optimed_0(1, :), y_l_elbow_optimed_0(2, :), y_l_elbow_optimed_0(3, :), 'r--'); 
plot3(y_r_elbow_optimed_0(1, :), y_r_elbow_optimed_0(2, :), y_r_elbow_optimed_0(3, :), 'r--'); % optimed DMP trajs

view(-45, 45);
title('Moved DMP trajs and Optimized DMP trajs', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 
legend([p1(1), p2(1)], 'Moved DMP trajs', 'Optimed DMP trajs', 'Location', 'NorthEastOutside', 'FontSize', 16);
%}

% Fig 5 - Comparison between Tracked trajs and Optimed DMP trajs
figure;

p1 = plot3(actual_l_wrist_pos_traj_0(1, :), actual_l_wrist_pos_traj_0(2, :), actual_l_wrist_pos_traj_0(3, :), 'g--'); hold on; grid on;
plot3(actual_r_wrist_pos_traj_0(1, :), actual_r_wrist_pos_traj_0(2, :), actual_r_wrist_pos_traj_0(3, :), 'g--'); 
plot3(actual_l_elbow_pos_traj_0(1, :), actual_l_elbow_pos_traj_0(2, :), actual_l_elbow_pos_traj_0(3, :), 'g--'); 
plot3(actual_r_elbow_pos_traj_0(1, :), actual_r_elbow_pos_traj_0(2, :), actual_r_elbow_pos_traj_0(3, :), 'g--');  % tracked (actually executed) trajs

p2 = plot3(y_l_wrist_optimed_0(1, :), y_l_wrist_optimed_0(2, :), y_l_wrist_optimed_0(3, :), 'r--'); 
plot3(y_r_wrist_optimed_0(1, :), y_r_wrist_optimed_0(2, :), y_r_wrist_optimed_0(3, :), 'r--'); 
plot3(y_l_elbow_optimed_0(1, :), y_l_elbow_optimed_0(2, :), y_l_elbow_optimed_0(3, :), 'r--'); 
plot3(y_r_elbow_optimed_0(1, :), y_r_elbow_optimed_0(2, :), y_r_elbow_optimed_0(3, :), 'r--'); % optimed DMP trajs

view(-45, 45);
title('First-round Tracked (actually executed) trajs and Optimized DMP trajs', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('x', 'FontSize', label_font_size); ylabel('y', 'FontSize', label_font_size); zlabel('z', 'FontSize', label_font_size); 
legend([p1(1), p2(1)], 'Tracked trajs', 'Optimed DMP trajs');%, 'Location', 'NorthEastOutside', 'FontSize', 16);


% Fig 6 - Comparison between Optimed DMP trajs and next-round tracked trajs
%{
figure;

p1 = plot3(y_l_wrist_optimed_0(1, :), y_l_wrist_optimed_0(2, :), y_l_wrist_optimed_0(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist_optimed_0(1, :), y_r_wrist_optimed_0(2, :), y_r_wrist_optimed_0(3, :), 'r--'); 
plot3(y_l_elbow_optimed_0(1, :), y_l_elbow_optimed_0(2, :), y_l_elbow_optimed_0(3, :), 'r--'); 
plot3(y_r_elbow_optimed_0(1, :), y_r_elbow_optimed_0(2, :), y_r_elbow_optimed_0(3, :), 'r--'); % optimed DMP trajs

p2 = plot3(actual_l_wrist_pos_traj_1(1, :), actual_l_wrist_pos_traj_1(2, :), actual_l_wrist_pos_traj_1(3, :), 'g--'); hold on; grid on;
plot3(actual_r_wrist_pos_traj_1(1, :), actual_r_wrist_pos_traj_1(2, :), actual_r_wrist_pos_traj_1(3, :), 'g--'); 
plot3(actual_l_elbow_pos_traj_1(1, :), actual_l_elbow_pos_traj_1(2, :), actual_l_elbow_pos_traj_1(3, :), 'g--'); 
plot3(actual_r_elbow_pos_traj_1(1, :), actual_r_elbow_pos_traj_1(2, :), actual_r_elbow_pos_traj_1(3, :), 'g--');  % tracked (actually executed) trajs

view(-45, 45);
title('First-round Optimized DMP trajs and Next-round Tracked trajs', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 
legend([p1(1), p2(1)], 'Optimed DMP trajs', 'Next-round Tracked trajs', 'Location', 'NorthEastOutside', 'FontSize', 16);
%}

% Fig 7 - Comparison between Optimed DMP trajs and last-round tracked trajs
figure;

p1 = plot3(y_l_wrist_optimed_0(1, :), y_l_wrist_optimed_0(2, :), y_l_wrist_optimed_0(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist_optimed_0(1, :), y_r_wrist_optimed_0(2, :), y_r_wrist_optimed_0(3, :), 'r--'); 
plot3(y_l_elbow_optimed_0(1, :), y_l_elbow_optimed_0(2, :), y_l_elbow_optimed_0(3, :), 'r--'); 
plot3(y_r_elbow_optimed_0(1, :), y_r_elbow_optimed_0(2, :), y_r_elbow_optimed_0(3, :), 'r--'); % optimed DMP trajs

p2 = plot3(actual_l_wrist_pos_traj_last(1, :), actual_l_wrist_pos_traj_last(2, :), actual_l_wrist_pos_traj_last(3, :), 'g--'); hold on; grid on;
plot3(actual_r_wrist_pos_traj_last(1, :), actual_r_wrist_pos_traj_last(2, :), actual_r_wrist_pos_traj_last(3, :), 'g--'); 
plot3(actual_l_elbow_pos_traj_last(1, :), actual_l_elbow_pos_traj_last(2, :), actual_l_elbow_pos_traj_last(3, :), 'g--'); 
plot3(actual_r_elbow_pos_traj_last(1, :), actual_r_elbow_pos_traj_last(2, :), actual_r_elbow_pos_traj_last(3, :), 'g--');  % tracked (actually executed) trajs

view(-45, 45);
title('First-round Optimized DMP trajs and Last-round Tracked trajs', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('x', 'FontSize', label_font_size); ylabel('y', 'FontSize', label_font_size); zlabel('z', 'FontSize', label_font_size); 
legend([p1(1), p2(1)], 'Optimed DMP trajs', 'Last-round Tracked trajs');%, 'Location', 'NorthEastOutside', 'FontSize', 16);


% Fig 8 - Comparison between First-round tracked trajs and last-round tracked trajs
figure;

p1 = plot3(actual_l_wrist_pos_traj_0(1, :), actual_l_wrist_pos_traj_0(2, :), actual_l_wrist_pos_traj_0(3, :), 'r--'); hold on; grid on;
plot3(actual_r_wrist_pos_traj_0(1, :), actual_r_wrist_pos_traj_0(2, :), actual_r_wrist_pos_traj_0(3, :), 'r--'); 
plot3(actual_l_elbow_pos_traj_0(1, :), actual_l_elbow_pos_traj_0(2, :), actual_l_elbow_pos_traj_0(3, :), 'r--'); 
plot3(actual_r_elbow_pos_traj_0(1, :), actual_r_elbow_pos_traj_0(2, :), actual_r_elbow_pos_traj_0(3, :), 'r--');  % First-round tracked (actually executed) trajs

p2 = plot3(actual_l_wrist_pos_traj_last(1, :), actual_l_wrist_pos_traj_last(2, :), actual_l_wrist_pos_traj_last(3, :), 'g--'); 
plot3(actual_r_wrist_pos_traj_last(1, :), actual_r_wrist_pos_traj_last(2, :), actual_r_wrist_pos_traj_last(3, :), 'g--'); 
plot3(actual_l_elbow_pos_traj_last(1, :), actual_l_elbow_pos_traj_last(2, :), actual_l_elbow_pos_traj_last(3, :), 'g--'); 
plot3(actual_r_elbow_pos_traj_last(1, :), actual_r_elbow_pos_traj_last(2, :), actual_r_elbow_pos_traj_last(3, :), 'g--');  % Last-round tracked (actually executed) trajs

view(-45, 45);
title('First-round Tracked trajs and Last-round Tracked trajs', 'FontSize', title_font_size, 'FontWeight', 'normal');
xlabel('x', 'FontSize', label_font_size); ylabel('y', 'FontSize', label_font_size); zlabel('z', 'FontSize', label_font_size); 
legend([p1(1), p2(1)], 'First-round Tracked trajs', 'Last-round Tracked trajs');%, 'Location', 'NorthEastOutside', 'FontSize', 16);




%% Final optimization results
%{
% load optimized DMP starts and goals
dmp_starts_goals = h5read(file_name, ['/', group_name, '/dmp_starts_goals_1']);
new_goal_lrw = dmp_starts_goals(1:3);
new_start_lrw = dmp_starts_goals(4:6);
new_goal_lew = dmp_starts_goals(7:9);
new_start_lew = dmp_starts_goals(10:12);
new_goal_rew = dmp_starts_goals(13:15);
new_start_rew = dmp_starts_goals(16:18);
new_goal_rw = dmp_starts_goals(19:21);
new_start_rw = dmp_starts_goals(22:24);

% plot pre-iteration target and result
y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);

y_r_wrist = y_rw;
y_l_wrist = y_rw + y_lrw;
y_r_elbow = y_rw + y_rew;
y_l_elbow = y_l_wrist + y_lew;

% 1 - with original
figure;
% original imitation data
plot3(l_wrist_pos(1, :), l_wrist_pos(2, :), l_wrist_pos(3, :), 'b--'); hold on; grid on;
plot3(r_wrist_pos(1, :), r_wrist_pos(2, :), r_wrist_pos(3, :), 'b--'); 
plot3(l_elbow_pos(1, :), l_elbow_pos(2, :), l_elbow_pos(3, :), 'b--'); 
plot3(r_elbow_pos(1, :), r_elbow_pos(2, :), r_elbow_pos(3, :), 'b--');  % original imitation data

% DMP trajectories expected to follow
plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--');
plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--');  % optimized DMP targets

% actually executed trajectories
% plot3(optimed_l_wrist_pos_traj(1, :), optimed_l_wrist_pos_traj(2, :), optimed_l_wrist_pos_traj(3, :), 'g--');
% plot3(optimed_r_wrist_pos_traj(1, :), optimed_r_wrist_pos_traj(2, :), optimed_r_wrist_pos_traj(3, :), 'g--'); 
% plot3(optimed_l_elbow_pos_traj(1, :), optimed_l_elbow_pos_traj(2, :), optimed_l_elbow_pos_traj(3, :), 'g--'); 
% plot3(optimed_r_elbow_pos_traj(1, :), optimed_r_elbow_pos_traj(2, :), optimed_r_elbow_pos_traj(3, :), 'g--');  % optimized q trajectories

view(-45, 45);
% title('Original, Desired and Final optimized trajectories', 'FontSize', 18);
title('Original, Final optimized trajectories', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 


% 2 - close up
figure;

% DMP trajectories expected to follow
plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--'); hold on; grid on;
plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--');  % optimized DMP targets

% actually executed trajectories
plot3(optimed_l_wrist_pos_traj(1, :), optimed_l_wrist_pos_traj(2, :), optimed_l_wrist_pos_traj(3, :), 'g--');
plot3(optimed_r_wrist_pos_traj(1, :), optimed_r_wrist_pos_traj(2, :), optimed_r_wrist_pos_traj(3, :), 'g--'); 
plot3(optimed_l_elbow_pos_traj(1, :), optimed_l_elbow_pos_traj(2, :), optimed_l_elbow_pos_traj(3, :), 'g--'); 
plot3(optimed_r_elbow_pos_traj(1, :), optimed_r_elbow_pos_traj(2, :), optimed_r_elbow_pos_traj(3, :), 'g--');  % optimized q trajectories

view(-45, 45);
title('Desired and Final optimized trajectories', 'FontSize', 18);
xlabel('x', 'FontSize', 18); ylabel('y', 'FontSize', 18); zlabel('z', 'FontSize', 18); 


%% Debug for choosing weights for q optimization (tracking), set a reasonable initial position to start with     
%{
% expected, desired trajectories
dmp_starts_goals = dmp_starts_goals_original;
new_goal_lrw = dmp_starts_goals(1:3);
new_start_lrw = dmp_starts_goals(4:6);
new_goal_lew = dmp_starts_goals(7:9);
new_start_lew = dmp_starts_goals(10:12);
new_goal_rew = dmp_starts_goals(13:15);
new_start_rew = dmp_starts_goals(16:18);
new_goal_rw = dmp_starts_goals(19:21);
new_start_rw = dmp_starts_goals(22:24);

% translate rw to desired position
rw_set_start = [0.5, -0.18, 0.5]';
rw_offset = rw_set_start - new_start_rw;
new_goal_rw = new_goal_rw + rw_offset;
new_start_rw = new_start_rw + rw_offset;


y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);

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

plot3(preiter_l_wrist_pos_traj(1, :), preiter_l_wrist_pos_traj(2, :), preiter_l_wrist_pos_traj(3, :), 'g--');
plot3(preiter_r_wrist_pos_traj(1, :), preiter_r_wrist_pos_traj(2, :), preiter_r_wrist_pos_traj(3, :), 'g--'); 
plot3(preiter_l_elbow_pos_traj(1, :), preiter_l_elbow_pos_traj(2, :), preiter_l_elbow_pos_traj(3, :), 'g--'); 
plot3(preiter_r_elbow_pos_traj(1, :), preiter_r_elbow_pos_traj(2, :), preiter_r_elbow_pos_traj(3, :), 'g--'); 

view(-45, 45);
% title('Original and reproduced trajectories');
title('Original, desired and pre-iteration optimized trajectories');
xlabel('x'); ylabel('y'); zlabel('z'); 
%}


%}



%% Display joint trajectories for analysis
%{
% plot the original joint trajectories
arm_traj_1 = h5read(file_name, ['/', group_name, '/arm_traj_1']);
plot_joints_subplots(arm_traj_1(1:7, :), 'Left Arm');
plot_joints_subplots(arm_traj_1(8:14, :), 'Right Arm');
plot_joints_subplots(arm_traj_1(15:26, :), 'Left Hand');
plot_joints_subplots(arm_traj_1(27:38, :), 'Right Hand');

% do smoothing and plot the comparison
arm_traj_1_smoothed = zeros(size(arm_traj_1, 1), size(1:0.5:num_datapoints, 2));
for i = 1 : size(arm_traj_1, 1)
    arm_traj_1_smoothed(i, :) = interp1(1:num_datapoints, arm_traj_1(i, :), 1:0.5:num_datapoints, 'spline'); % cubic?
end
plot_joints_comp_subplots(arm_traj_1(1:7, :), arm_traj_1_smoothed(1:7, :), 'Left Arm', 'Ori Traj', 'Smoothed Traj');
plot_joints_comp_subplots(arm_traj_1(8:14, :), arm_traj_1_smoothed(8:14, :), 'Right Arm', 'Ori Traj', 'Smoothed Traj');
plot_joints_comp_subplots(arm_traj_1(15:26, :), arm_traj_1_smoothed(15:26, :), 'Left Hand', 'Ori Traj', 'Smoothed Traj');
plot_joints_comp_subplots(arm_traj_1(27:38, :), arm_traj_1_smoothed(27:38, :), 'Right Hand', 'Ori Traj', 'Smoothed Traj');

%}





