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
l_wrist_pos_cost_history = h5read(file_name, ['/', group_name, '/l_wrist_pos_cost_history']);
r_wrist_pos_cost_history = h5read(file_name, ['/', group_name, '/r_wrist_pos_cost_history']);
wrist_ori_cost_history = h5read(file_name, ['/', group_name, '/wrist_ori_cost_history']);
elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/elbow_pos_cost_history']);
l_elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/l_elbow_pos_cost_history']);
r_elbow_pos_cost_history = h5read(file_name, ['/', group_name, '/r_elbow_pos_cost_history']);
finger_cost_history = h5read(file_name, ['/', group_name, '/finger_cost_history']);
similarity_cost_history = h5read(file_name, ['/', group_name, '/similarity_cost_history']);
smoothness_cost_history = h5read(file_name, ['/', group_name, '/smoothness_cost_history']);
col_cost_history = h5read(file_name, ['/', group_name, '/col_cost_history']);
pos_limit_cost_history = h5read(file_name, ['/', group_name, '/pos_limit_cost_history']);
dmp_scale_cost_history = h5read(file_name, ['/', group_name, '/dmp_scale_cost_history']);
dmp_orien_cost_history = h5read(file_name, ['/', group_name, '/dmp_orien_cost_history']);

% jacobians for DMP starts and goals
sim_jacobian_history = h5read(file_name, ['/', group_name, '/sim_jacobian_history']);
track_jacobian_history = h5read(file_name, ['/', group_name, '/track_jacobian_history']);
orien_jacobian_history = h5read(file_name, ['/', group_name, '/orien_jacobian_history']);
scale_jacobian_history = h5read(file_name, ['/', group_name, '/scale_jacobian_history']);
% update history of DMP
dmp_update_history = h5read(file_name, ['/', group_name, '/dmp_update_history']);


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
plot((1:size(l_wrist_pos_cost_history, 2))*per_iteration, sum(l_wrist_pos_cost_history), 'b-'); hold on; grid on;
plot((1:size(l_wrist_pos_cost_history, 2))*per_iteration, sum(l_wrist_pos_cost_history), 'bo');
title('History of left wrist position cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(r_wrist_pos_cost_history, 2))*per_iteration, sum(r_wrist_pos_cost_history), 'b-'); hold on; grid on;
plot((1:size(r_wrist_pos_cost_history, 2))*per_iteration, sum(r_wrist_pos_cost_history), 'bo');
title('History of right wrist position cost');
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
plot((1:size(dmp_orien_cost_history, 2))*per_iteration, dmp_orien_cost_history, 'b-'); hold on; grid on;
plot((1:size(dmp_orien_cost_history, 2))*per_iteration, dmp_orien_cost_history, 'bo');
title('History of dmp orientation cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(dmp_scale_cost_history, 2))*per_iteration, dmp_scale_cost_history, 'b-'); hold on; grid on;
plot((1:size(dmp_scale_cost_history, 2))*per_iteration, dmp_scale_cost_history, 'bo');
title('History of dmp scale cost');
xlabel('Iterations'); ylabel('Cost Value'); 

figure;
plot((1:size(similarity_cost_history, 2))*per_iteration, similarity_cost_history, 'b-'); hold on; grid on;
plot((1:size(similarity_cost_history, 2))*per_iteration, similarity_cost_history, 'bo');
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

num_records = size(orien_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(orien_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of DMPOrienConstraint Jacobians');
xlabel('Iterations'); ylabel('Gradients magnitude'); 

num_records = size(scale_jacobian_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(scale_jacobian_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of DMPScaleConstraint Jacobians');
xlabel('Iterations'); ylabel('Gradients magnitude'); 

% should use norm() instead of sum(), which is more reasonable
num_records = size(dmp_update_history, 2);
jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    jacobian_norm(n) = norm(dmp_update_history(:, n));
end
figure;
plot((1:num_records)*per_iteration, jacobian_norm, 'b-'); hold on; grid on;
plot((1:num_records)*per_iteration, jacobian_norm, 'bo');
title('History of DMP updates');
xlabel('Iterations'); ylabel('Norm of updates on all DOF'); 



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




%% Debug: try to establish a constraint for DMP starts and goals
% get goals and starts
lrw_goal_ori = dmp_starts_goals_original(1:3);
lrw_start_ori = dmp_starts_goals_original(4:6);
lew_goal_ori = dmp_starts_goals_original(7:9);
lew_start_ori = dmp_starts_goals_original(10:12);
rew_goal_ori = dmp_starts_goals_original(13:15);
rew_start_ori = dmp_starts_goals_original(16:18);
rw_goal_ori = dmp_starts_goals_original(19:21);
rw_start_ori = dmp_starts_goals_original(22:24);

lrw_goal_new = dmp_starts_goals(1:3);
lrw_start_new = dmp_starts_goals(4:6);
lew_goal_new = dmp_starts_goals(7:9);
lew_start_new = dmp_starts_goals(10:12);
rew_goal_new = dmp_starts_goals(13:15);
rew_start_new = dmp_starts_goals(16:18);
rw_goal_new = dmp_starts_goals(19:21);
rw_start_new = dmp_starts_goals(22:24);

% get vectors pointing from starts to goals
lrw_vec_ori = lrw_goal_ori - lrw_start_ori;
lew_vec_ori = lew_goal_ori - lew_start_ori;
rew_vec_ori = rew_goal_ori - rew_start_ori;
rw_vec_ori = rw_goal_ori - rw_start_ori;

lrw_vec_new = lrw_goal_new - lrw_start_new;
lew_vec_new = lew_goal_new - lew_start_new;
rew_vec_new = rew_goal_new - rew_start_new;
rw_vec_new = rw_goal_new - rw_start_new;

lrw_ratio = norm(lrw_vec_new) / norm(lrw_vec_ori);
lew_ratio = norm(lew_vec_new) / norm(lew_vec_ori);
rew_ratio = norm(rew_vec_new) / norm(rew_vec_ori);
rw_ratio = norm(rw_vec_new) / norm(rw_vec_ori);

lrw_th = acos(lrw_vec_new'*lrw_vec_ori/norm(lrw_vec_new)/norm(lrw_vec_ori)) * 180 / pi;
lew_th = acos(lew_vec_new'*lew_vec_ori/norm(lew_vec_new)/norm(lew_vec_ori)) * 180 / pi;
rew_th = acos(rew_vec_new'*rew_vec_ori/norm(rew_vec_new)/norm(rew_vec_ori)) * 180 / pi;
rw_th = acos(rw_vec_new'*rw_vec_ori/norm(rw_vec_new)/norm(rw_vec_ori)) * 180 / pi;



%% Debug: analyze jacobian history
% raw data, 24-dim
figure;
p1 = plot((1:size(orien_jacobian_history, 2))*per_iteration, orien_jacobian_history, 'b-'); hold on; grid on;
plot((1:size(orien_jacobian_history, 2))*per_iteration, orien_jacobian_history, 'bo'); % 1
p2 = plot((1:size(scale_jacobian_history, 2))*per_iteration, scale_jacobian_history, 'r-'); 
plot((1:size(scale_jacobian_history, 2))*per_iteration, scale_jacobian_history, 'ro'); % 2
p3 = plot((1:size(track_jacobian_history, 2))*per_iteration, track_jacobian_history, 'g-'); 
plot((1:size(track_jacobian_history, 2))*per_iteration, track_jacobian_history, 'go'); % 3
p4 = plot((1:size(sim_jacobian_history, 2))*per_iteration, sim_jacobian_history, 'm-'); 
plot((1:size(sim_jacobian_history, 2))*per_iteration, sim_jacobian_history, 'mo'); % 4 
title('History of Jacobians w.r.t DMP starts and goals');
xlabel('Iterations'); ylabel('Jacobians'); 
legend([p1(1), p2(1), p3(1), p4(1)], 'orien\_jacobian', 'scale\_jacobian', 'track\_jacobian', 'sim\_jacobian', 'Location', 'NorthEastOutside');
% 'track_jacobian', 'sim_jacobian', 'Location', 'NorthEastOutside');


% norms
num_records = size(track_jacobian_history, 2);
track_jacobian_norm = zeros(1, num_records);
sim_jacobian_norm = zeros(1, num_records);
orien_jacobian_norm = zeros(1, num_records);
scale_jacobian_norm = zeros(1, num_records);
for n = 1 : num_records
    track_jacobian_norm(n) = norm(track_jacobian_history(:, n));
    sim_jacobian_norm(n) = norm(sim_jacobian_history(:, n));
    orien_jacobian_norm(n) = norm(orien_jacobian_history(:, n));
    scale_jacobian_norm(n) = norm(scale_jacobian_history(:, n));
end
figure;
p1 = plot((1:num_records)*per_iteration, orien_jacobian_norm, 'b-'); hold on; grid on;
p2 = plot((1:num_records)*per_iteration, scale_jacobian_norm, 'r-');
p3 = plot((1:num_records)*per_iteration, track_jacobian_norm, 'g-');
p4 = plot((1:num_records)*per_iteration, sim_jacobian_norm, 'm-');
title('History of Norms of DMP Jacobians');
xlabel('Iterations'); ylabel('Norms of jacobians'); 
legend([p1, p2, p3, p4], 'orien\_jacobian', 'scale\_jacobian', 'track\_jacobian', 'sim\_jacobian', 'Location', 'NorthEastOutside');



