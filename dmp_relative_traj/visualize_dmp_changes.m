%% This script displays the change of DMP generated and superimposed trajectories, across rounds of DMP optimization.

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj


%% Prep
file_name = '../motion-retargeting/dmp_optimize_results-.h5';
dmp_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
group_name = 'baozhu_1';

num_resampled_points = 50;

kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
dt = 0.04; %1/num_datapoints; %Duration of time step


%% Load data
rw_goal_start_history = h5read(file_name, ['/', group_name, '/rw_goal_start_history']);
lrw_goal_start_history = h5read(file_name, ['/', group_name, '/lrw_goal_start_history']);
lew_goal_start_history = h5read(file_name, ['/', group_name, '/lew_goal_start_history']);
rew_goal_start_history = h5read(file_name, ['/', group_name, '/rew_goal_start_history']);

num_iters = size(lrw_goal_start_history, 2);


%% Load DMP coefficients
Mu_lrw = h5read(dmp_file_name, ['/', group_name, '/Mu_lrw']);
Mu_lew = h5read(dmp_file_name, ['/', group_name, '/Mu_lew']);
Mu_rew = h5read(dmp_file_name, ['/', group_name, '/Mu_rew']);
Mu_rw = h5read(dmp_file_name, ['/', group_name, '/Mu_rw']);

Sigma_lrw = h5read(dmp_file_name, ['/', group_name, '/Sigma_lrw']);
Sigma_lew = h5read(dmp_file_name, ['/', group_name, '/Sigma_lew']);
Sigma_rew = h5read(dmp_file_name, ['/', group_name, '/Sigma_rew']);
Sigma_rw = h5read(dmp_file_name, ['/', group_name, '/Sigma_rw']);

Weights_lrw = h5read(dmp_file_name, ['/', group_name, '/Weights_lrw']);
Weights_lew = h5read(dmp_file_name, ['/', group_name, '/Weights_lew']);
Weights_rew = h5read(dmp_file_name, ['/', group_name, '/Weights_rew']);
Weights_rw = h5read(dmp_file_name, ['/', group_name, '/Weights_rw']);


%% Display the changes in 3d view
%{
figure;
for i = 1 : num_iters
    % get DMP starts and goals
    new_goal_lrw = lrw_goal_start_history(1:3, i);
    new_start_lrw = lrw_goal_start_history(4:6, i);
    new_goal_lew = lew_goal_start_history(1:3, i);
    new_start_lew = lew_goal_start_history(4:6, i);
    new_goal_rew = rew_goal_start_history(1:3, i);
    new_start_rew = rew_goal_start_history(4:6, i);
    new_goal_rw = rw_goal_start_history(1:3, i);
    new_start_rw = rw_goal_start_history(4:6, i);
    % generate DMP trajs
    y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, num_resampled_points, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
    y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, num_resampled_points, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
    y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, num_resampled_points, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
    y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, num_resampled_points, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);
    % add up to construct wrist and elbow trajs
    y_r_wrist = y_rw;
    y_l_wrist = y_rw + y_lrw;
    y_r_elbow = y_rw + y_rew;
    y_l_elbow = y_l_wrist + y_lew;
    % plot it
    plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--'); hold on; grid on;
    plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
    plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
    plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--'); % DMP generated movement
    view(-45, 45);
    axis([0.2, 0.7, -0.4, 0.4, 0.1, 0.7]);
    title(['DMP generated trajectories - Round ', num2str(i)], 'FontSize', 16);
    xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 
    hold off;
    pause;
end
%}

%% Plot the changes in 3-dim split view
figure;

for i = 1 : num_iters
    % get DMP starts and goals
    new_goal_lrw = lrw_goal_start_history(1:3, i);
    new_start_lrw = lrw_goal_start_history(4:6, i);
    new_goal_lew = lew_goal_start_history(1:3, i);
    new_start_lew = lew_goal_start_history(4:6, i);
    new_goal_rew = rew_goal_start_history(1:3, i);
    new_start_rew = rew_goal_start_history(4:6, i);
    new_goal_rw = rw_goal_start_history(1:3, i);
    new_start_rw = rw_goal_start_history(4:6, i);
    % generate DMP trajs
    y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, num_resampled_points, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
    y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, num_resampled_points, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
    y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, num_resampled_points, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
    y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, num_resampled_points, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);
    % add up to construct wrist and elbow trajs
    y_r_wrist = y_rw;
    y_l_wrist = y_rw + y_lrw;
    y_r_elbow = y_rw + y_rew;
    y_l_elbow = y_l_wrist + y_lew;
    % plot in split view
    % l wrist
    subplot(3, 4, 1);
    plot(1:num_resampled_points, y_l_wrist(1,:), 'r-'); hold on; grid on;
    title('l wrsit');
    subplot(3, 4, 5); 
    plot(1:num_resampled_points, y_l_wrist(2,:), 'r-'); hold on; grid on;
    subplot(3, 4, 9);
    plot(1:num_resampled_points, y_l_wrist(3,:), 'r-'); hold on; grid on;
    % r wrist
    subplot(3, 4, 2);
    plot(1:num_resampled_points, y_r_wrist(1,:), 'r-'); hold on; grid on;
    title('r wrist');
    subplot(3, 4, 6);
    plot(1:num_resampled_points, y_r_wrist(2,:), 'r-'); hold on; grid on;
    subplot(3, 4, 10);
    plot(1:num_resampled_points, y_r_wrist(3,:), 'r-'); hold on; grid on;
    % l elbow
    subplot(3, 4, 3);
    plot(1:num_resampled_points, y_l_elbow(1,:), 'r-'); hold on; grid on;
    title('l elbow');
    subplot(3, 4, 7);
    plot(1:num_resampled_points, y_l_elbow(2,:), 'r-'); hold on; grid on;
    subplot(3, 4, 11);
    plot(1:num_resampled_points, y_l_elbow(3,:), 'r-'); hold on; grid on;
    % r elbow
    subplot(3, 4, 4);
    plot(1:num_resampled_points, y_r_elbow(1,:), 'r-'); hold on; grid on;
    title('r elbow');
    subplot(3, 4, 8);
    plot(1:num_resampled_points, y_r_elbow(2,:), 'r-'); hold on; grid on;
    subplot(3, 4, 12);
    plot(1:num_resampled_points, y_r_elbow(3,:), 'r-'); hold on; grid on;
    
    sgtitle(['DMP generated trajectories - Round ', num2str(i)], 'FontSize', 16);
    hold off;
    pause;
end








