%% This script displays the change of DMP generated and superimposed trajectories, across rounds of DMP optimization.

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj


%% Prep
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
dmp_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
group_name = 'gun_2';

num_resampled_points = 50;

kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
dt = 0.04; %1/num_datapoints; %Duration of time step


%% Load data from liweijie's result (liweijie)
%{
rw_goal_start_history = h5read(file_name, ['/', group_name, '/rw_goal_start_history']);
lrw_goal_start_history = h5read(file_name, ['/', group_name, '/lrw_goal_start_history']);
lew_goal_start_history = h5read(file_name, ['/', group_name, '/lew_goal_start_history']);
rew_goal_start_history = h5read(file_name, ['/', group_name, '/rew_goal_start_history']);

num_iters = size(lrw_goal_start_history, 2);
%}

%% Arrange moved and optimized DMP starts and goals into history data (liangyuwei)
%
% initialization
max_round = h5read(file_name, ['/', group_name, '/max_round']); % for ease of display
num_iters = 1 + 2 * max_round;
dmp_starts_goals_history = zeros(24, num_iters);
% 1 - initial
dmp_starts_goals_history(:, 1) = h5read(file_name, ['/', group_name, '/dmp_starts_goals_initial']);
% 2 - iterate to include all successive changes
for n = 0 : max_round-1
    % moved DMP starts and goals
    dmp_starts_goals_history(:, n*2+2) = h5read(file_name, ['/', group_name, '/dmp_starts_goals_moved_', num2str(n)]);
    % optimized DMP starts and goals
    dmp_starts_goals_history(:, n*2+3) = h5read(file_name, ['/', group_name, '/dmp_starts_goals_optimed_', num2str(n)]);
end


% split into four parts
lrw_goal_start_history = dmp_starts_goals_history(1:6, :);
lew_goal_start_history = dmp_starts_goals_history(7:12, :);
rew_goal_start_history = dmp_starts_goals_history(13:18, :);
rw_goal_start_history = dmp_starts_goals_history(19:24, :);
%}

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
% get trajectories
y_r_wrist_history = zeros(3, num_resampled_points, num_iters);
y_l_wrist_history = zeros(3, num_resampled_points, num_iters);
y_r_elbow_history = zeros(3, num_resampled_points, num_iters);
y_l_elbow_history = zeros(3, num_resampled_points, num_iters);
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
    y_r_wrist_history(:, :, i) = y_rw;
    y_l_wrist_history(:, :, i) = y_rw + y_lrw;
    y_r_elbow_history(:, :, i) = y_rw + y_rew;
    y_l_elbow_history(:, :, i) = y_l_wrist_history(:, :, i) + y_lew;    
end

% get the area of motion
tmp_trajectories_max = [];
tmp_trajectories_min = [];
for i = 1 : num_iters
    % max
    tmp_trajectories_max = [tmp_trajectories_max, max(y_l_wrist_history(:, :, i), [], 2)];
    tmp_trajectories_max = [tmp_trajectories_max, max(y_l_elbow_history(:, :, i), [], 2)];
    tmp_trajectories_max = [tmp_trajectories_max, max(y_r_wrist_history(:, :, i), [], 2)];
    tmp_trajectories_max = [tmp_trajectories_max, max(y_r_elbow_history(:, :, i), [], 2)];
    % min
    tmp_trajectories_min = [tmp_trajectories_min, min(y_l_wrist_history(:, :, i), [], 2)];
    tmp_trajectories_min = [tmp_trajectories_min, min(y_l_elbow_history(:, :, i), [], 2)];
    tmp_trajectories_min = [tmp_trajectories_min, min(y_r_wrist_history(:, :, i), [], 2)];
    tmp_trajectories_min = [tmp_trajectories_min, min(y_r_elbow_history(:, :, i), [], 2)];
end
xyz_max = max(tmp_trajectories_max, [], 2);
xyz_min = min(tmp_trajectories_min, [], 2);

% plot the change of DMP trajectories
figure;
for i = 1 : num_iters
    % obtain the current trajectories
    y_r_wrist = y_r_wrist_history(:, :, i);
    y_l_wrist = y_l_wrist_history(:, :, i);
    y_r_elbow = y_r_elbow_history(:, :, i);
    y_l_elbow = y_l_elbow_history(:, :, i);
    % plot it
    plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--'); hold on; grid on;
    plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
    plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
    plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--'); % DMP generated movement
    view(-45, 45);
    axis([xyz_min(1)-0.1, xyz_max(1)+0.1, ...
          xyz_min(2)-0.1, xyz_max(2)+0.1, ...
          xyz_min(3)-0.1, xyz_max(3)+0.1]);
    title(['DMP generated trajectories - Round ', num2str(i)], 'FontSize', 16);
    xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 
    hold off;
    pause;
end
%}


%% Plot the changes in 3-dim split view
% obtain the range for each trajectory
tmp_lw_max = []; tmp_lw_min = [];
tmp_le_max = []; tmp_le_min = [];
tmp_rw_max = []; tmp_rw_min = [];
tmp_re_max = []; tmp_re_min = [];
for i = 1 : num_iters
    % lw
    tmp_lw_max = [tmp_lw_max, max(y_l_wrist_history(:, :, i), [], 2)];
    tmp_lw_min = [tmp_lw_min, min(y_l_wrist_history(:, :, i), [], 2)];
    % le
    tmp_le_max = [tmp_le_max, max(y_l_elbow_history(:, :, i), [], 2)];
    tmp_le_min = [tmp_le_min, min(y_l_elbow_history(:, :, i), [], 2)];
    % rw
    tmp_rw_max = [tmp_rw_max, max(y_r_wrist_history(:, :, i), [], 2)];
    tmp_rw_min = [tmp_rw_min, min(y_r_wrist_history(:, :, i), [], 2)];
    % re
    tmp_re_max = [tmp_re_max, max(y_r_elbow_history(:, :, i), [], 2)];
    tmp_re_min = [tmp_re_min, min(y_r_elbow_history(:, :, i), [], 2)];
end
tmp_lw_max = max(tmp_lw_max, [], 2); tmp_lw_min = min(tmp_lw_min, [], 2);
tmp_le_max = max(tmp_le_max, [], 2); tmp_le_min = min(tmp_le_min, [], 2);
tmp_rw_max = max(tmp_rw_max, [], 2); tmp_rw_min = min(tmp_rw_min, [], 2);
tmp_re_max = max(tmp_re_max, [], 2); tmp_re_min = min(tmp_re_min, [], 2);

% display 
d = 0; %0.01; % offset for display
figure;
for i = 1 : num_iters
    % obtain the current trajectories
    y_r_wrist = y_r_wrist_history(:, :, i);
    y_l_wrist = y_l_wrist_history(:, :, i);
    y_r_elbow = y_r_elbow_history(:, :, i);
    y_l_elbow = y_l_elbow_history(:, :, i);
    
        % plot in split view
        % 1 - l wrist
        % x 
        subplot(3, 4, 1);
        plot(1:num_resampled_points, y_l_wrist(1,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_lw_min(1)-d, tmp_lw_max(1)+d]);
        xlabel('x');
        title('l wrsit');
        % y
        subplot(3, 4, 5); 
        plot(1:num_resampled_points, y_l_wrist(2,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_lw_min(2)-d, tmp_lw_max(2)+d]);
        xlabel('y');
        % z 
        subplot(3, 4, 9);
        plot(1:num_resampled_points, y_l_wrist(3,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_lw_min(3)-d, tmp_lw_max(3)+d]);
        xlabel('z');
        % 2 - r wrist
        % x
        subplot(3, 4, 2);
        plot(1:num_resampled_points, y_r_wrist(1,:), 'r-'); hold on; grid on;
        title('r wrist');
    %     axis([0, num_resampled_points+1, tmp_rw_min(1)-d, tmp_rw_max(1)+d]);    
        xlabel('x');
        % y 
        subplot(3, 4, 6);
        plot(1:num_resampled_points, y_r_wrist(2,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_rw_min(2)-d, tmp_rw_max(2)+d]);    
        xlabel('y');
        % z 
        subplot(3, 4, 10);
        plot(1:num_resampled_points, y_r_wrist(3,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_rw_min(3)-d, tmp_rw_max(3)+d]);    
        xlabel('z');
        % 3 - l elbow
        % x
        subplot(3, 4, 3);
        plot(1:num_resampled_points, y_l_elbow(1,:), 'r-'); hold on; grid on;
        title('l elbow');
    %     axis([0, num_resampled_points+1, tmp_le_min(1)-d, tmp_le_max(1)+d]);    
        xlabel('x');
        % y
        subplot(3, 4, 7);
        plot(1:num_resampled_points, y_l_elbow(2,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_le_min(2)-d, tmp_le_max(2)+d]);    
        xlabel('y');
        % z
        subplot(3, 4, 11);
        plot(1:num_resampled_points, y_l_elbow(3,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_le_min(3)-d, tmp_le_max(3)+d]);    
        xlabel('z');
        % 4 - r elbow
        % x
        subplot(3, 4, 4);
        plot(1:num_resampled_points, y_r_elbow(1,:), 'r-'); hold on; grid on;
        title('r elbow');
    %     axis([0, num_resampled_points+1, tmp_re_min(1)-d, tmp_re_max(1)+d]);    
        xlabel('x');
        % y
        subplot(3, 4, 8);
        plot(1:num_resampled_points, y_r_elbow(2,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_re_min(2)-d, tmp_re_max(2)+d]);    
        xlabel('y');
        % z
        subplot(3, 4, 12);
        plot(1:num_resampled_points, y_r_elbow(3,:), 'r-'); hold on; grid on;
    %     axis([0, num_resampled_points+1, tmp_re_min(3)-d, tmp_re_max(3)+d]);    
        xlabel('z');
 
        sgtitle(['DMP generated trajectories - Round ', num2str(i)], 'FontSize', 16);
        hold off;
        pause;

end


