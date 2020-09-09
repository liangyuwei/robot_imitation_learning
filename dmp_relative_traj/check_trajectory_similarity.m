%% check trajectory similarity



%% Prep
clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj
addpath('./'); % add all to include class folders

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
group_name = 'gun_2';%'kai_2';%'baozhu_1';%'fengren_1';

num_datapoints = 50;

max_round = h5read(file_name, ['/', group_name, '/max_round']); % for ease of display


%% Load data
% 1 - human demonstrations
% absolute
l_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/l_wrist_pos']);
r_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/r_wrist_pos']);
l_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/l_elbow_pos']);
r_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/r_elbow_pos']);

% relative
lrw_pos_human = l_wrist_pos_human - r_wrist_pos_human;
lew_pos_human = l_elbow_pos_human - l_wrist_pos_human;
rew_pos_human = r_elbow_pos_human - r_wrist_pos_human;

% normalize to [0,1] x [0,1] x [0,1]
for i = 1 : 3
    % absolute
    l_wrist_pos_human(i, :) = mapminmax(l_wrist_pos_human(i, :), 0, 1);
    r_wrist_pos_human(i, :) = mapminmax(r_wrist_pos_human(i, :), 0, 1);
    l_elbow_pos_human(i, :) = mapminmax(l_elbow_pos_human(i, :), 0, 1);
    r_elbow_pos_human(i, :) = mapminmax(r_elbow_pos_human(i, :), 0, 1);
    % relative
%     lrw_pos_human(i, :) = mapminmax(lrw_pos_human(i, :), 0, 1);
%     lew_pos_human(i, :) = mapminmax(lew_pos_human(i, :), 0, 1);
%     rew_pos_human(i, :) = mapminmax(rew_pos_human(i, :), 0, 1);
end





%% Calculate Frechet distance
%
% initialize frechet distance history
% absolute
frdist_l_wrist_pos = zeros(1, max_round+1);
frdist_l_elbow_pos = zeros(1, max_round+1);
frdist_r_wrist_pos = zeros(1, max_round+1);
frdist_r_elbow_pos = zeros(1, max_round+1);
% relative
frdist_lrw_pos = zeros(1, max_round+1);
frdist_lew_pos = zeros(1, max_round+1);
frdist_rew_pos = zeros(1, max_round+1);

% iterate over optimization results of different rounds
for i = 1 : max_round + 1 
    % read actually tracked trajectories (absolute)
    actual_l_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_', num2str(i-1)]);
    actual_r_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_', num2str(i-1)]);
    actual_l_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_', num2str(i-1)]);
    actual_r_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_', num2str(i-1)]);
    % compute actually tracked trajectories (relative)
    actual_lrw_pos_traj = actual_l_wrist_pos_traj - actual_r_wrist_pos_traj;
    actual_lew_pos_traj = actual_l_elbow_pos_traj - actual_l_wrist_pos_traj;
    actual_rew_pos_traj = actual_r_elbow_pos_traj - actual_r_wrist_pos_traj;
    
    % normalize (absolute)
    for j = 1 : 3
        % absolute
        actual_l_wrist_pos_traj(j, :) = mapminmax(actual_l_wrist_pos_traj(j, :), 0, 1);
        actual_r_wrist_pos_traj(j, :) = mapminmax(actual_r_wrist_pos_traj(j, :), 0, 1);
        actual_l_elbow_pos_traj(j, :) = mapminmax(actual_l_elbow_pos_traj(j, :), 0, 1);
        actual_r_elbow_pos_traj(j, :) = mapminmax(actual_r_elbow_pos_traj(j, :), 0, 1);
        % relative
%         actual_lrw_pos_traj(j, :) = mapminmax(actual_lrw_pos_traj(j, :), 0, 1);
%         actual_lew_pos_traj(j, :) = mapminmax(actual_lew_pos_traj(j, :), 0, 1);
%         actual_rew_pos_traj(j, :) = mapminmax(actual_rew_pos_traj(j, :), 0, 1);
    end
    
    % compute discrete Frechet distance (absolute)
    frdist_l_wrist_pos(i) = frdist(l_wrist_pos_human, actual_l_wrist_pos_traj);
    frdist_l_elbow_pos(i) = frdist(l_elbow_pos_human, actual_l_elbow_pos_traj);
    frdist_r_wrist_pos(i) = frdist(r_wrist_pos_human, actual_r_wrist_pos_traj);
    frdist_r_elbow_pos(i) = frdist(r_elbow_pos_human, actual_r_elbow_pos_traj);
    % compute discrete Frechet distance (relative)
    frdist_lrw_pos(i) = frdist(lrw_pos_human, actual_lrw_pos_traj);    
    frdist_lew_pos(i) = frdist(lew_pos_human, actual_lew_pos_traj);    
    frdist_rew_pos(i) = frdist(rew_pos_human, actual_rew_pos_traj);    
end


%% Display all Frechet distance history in one figure
figure;
sgtitle('History of discrete Frechet distances across different optimization rounds');
% lw
subplot(2, 12, [1,2,3]);
plot(1:max_round+1, frdist_l_wrist_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Left Wrist');
% le
subplot(2, 12, [4,5,6]);
plot(1:max_round+1, frdist_l_elbow_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Left Elbow');
% rw
subplot(2, 12, [7, 8, 9]);
plot(1:max_round+1, frdist_r_wrist_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Right Wrist');
% re
subplot(2, 12, [10,11,12]);
plot(1:max_round+1, frdist_r_elbow_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Right Elbow');

% lrw
subplot(2, 12, [13,14,15,16]);
plot(1:max_round+1, frdist_lrw_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Left-Right Wrist Relative');
% lew
subplot(2, 12, [17,18,19,20]);
plot(1:max_round+1, frdist_lew_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Left Elbow-Wrist Relative');
% rew
subplot(2, 12, [21,22,23,24]);
plot(1:max_round+1, frdist_rew_pos, 'b-'); hold on; grid on;
xlabel('Round'); ylabel('Dist');
title('Right Elbow-Wrist Relative');
%}

%% Display the original trajectories and the last-round tracked trajectories (which is best_q indeed) in split view
% read actually tracked trajectories (absolute)
actual_l_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_', num2str(max_round)]);
actual_r_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_', num2str(max_round)]);
actual_l_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_', num2str(max_round)]);
actual_r_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_', num2str(max_round)]);
% compute actually tracked trajectories (relative)
actual_lrw_pos_traj = actual_l_wrist_pos_traj - actual_r_wrist_pos_traj;
actual_lew_pos_traj = actual_l_elbow_pos_traj - actual_l_wrist_pos_traj;
actual_rew_pos_traj = actual_r_elbow_pos_traj - actual_r_wrist_pos_traj;

% normalize (absolute)
for j = 1 : 3
    % absolute
    actual_l_wrist_pos_traj(j, :) = mapminmax(actual_l_wrist_pos_traj(j, :), 0, 1);
    actual_r_wrist_pos_traj(j, :) = mapminmax(actual_r_wrist_pos_traj(j, :), 0, 1);
    actual_l_elbow_pos_traj(j, :) = mapminmax(actual_l_elbow_pos_traj(j, :), 0, 1);
    actual_r_elbow_pos_traj(j, :) = mapminmax(actual_r_elbow_pos_traj(j, :), 0, 1);
    % relative
%     actual_lrw_pos_traj(j, :) = mapminmax(actual_lrw_pos_traj(j, :), 0, 1);
%     actual_lew_pos_traj(j, :) = mapminmax(actual_lew_pos_traj(j, :), 0, 1);
%     actual_rew_pos_traj(j, :) = mapminmax(actual_rew_pos_traj(j, :), 0, 1);
end
    
% align the length
l_wrist_pos_human_aligned = zeros(3, num_datapoints);
l_elbow_pos_human_aligned = zeros(3, num_datapoints);
r_wrist_pos_human_aligned = zeros(3, num_datapoints);
r_elbow_pos_human_aligned = zeros(3, num_datapoints);
lrw_pos_human_aligned = zeros(3, num_datapoints);
lew_pos_human_aligned = zeros(3, num_datapoints);
rew_pos_human_aligned = zeros(3, num_datapoints);
original_num_datapoints = size(l_wrist_pos_human, 2);
original_timestamps = linspace(0, 1, original_num_datapoints);
target_timestamps = linspace(0, 1, num_datapoints);
for i = 1 : 3
    l_wrist_pos_human_aligned(i, :) = interp1(original_timestamps, l_wrist_pos_human(i, :), target_timestamps, 'linear');
    l_elbow_pos_human_aligned(i, :) = interp1(original_timestamps, l_elbow_pos_human(i, :), target_timestamps, 'linear');
    r_wrist_pos_human_aligned(i, :) = interp1(original_timestamps, r_wrist_pos_human(i, :), target_timestamps, 'linear');
    r_elbow_pos_human_aligned(i, :) = interp1(original_timestamps, r_elbow_pos_human(i, :), target_timestamps, 'linear');
    lrw_pos_human_aligned(i, :) = interp1(original_timestamps, lrw_pos_human(i, :), target_timestamps, 'linear');
    lew_pos_human_aligned(i, :) = interp1(original_timestamps, lew_pos_human(i, :), target_timestamps, 'linear');
    rew_pos_human_aligned(i, :) = interp1(original_timestamps, rew_pos_human(i, :), target_timestamps, 'linear');
end

% display
% absolute
plot_3d_subplots(l_wrist_pos_human_aligned, actual_l_wrist_pos_traj, 'Left Wrist', 'Human', 'Robot');
plot_3d_subplots(l_elbow_pos_human_aligned, actual_l_elbow_pos_traj, 'Left Elbow', 'Human', 'Robot');
plot_3d_subplots(r_wrist_pos_human_aligned, actual_r_wrist_pos_traj, 'Right Wrist', 'Human', 'Robot');
plot_3d_subplots(r_elbow_pos_human_aligned, actual_r_elbow_pos_traj, 'Right Elbow', 'Human', 'Robot');
% relative
plot_3d_subplots(lrw_pos_human_aligned, actual_lrw_pos_traj, 'Left-Right Wrist', 'Human', 'Robot');
plot_3d_subplots(lew_pos_human_aligned, actual_lew_pos_traj, 'Left Elbow-Wrist', 'Human', 'Robot');
plot_3d_subplots(rew_pos_human_aligned, actual_rew_pos_traj, 'Right Elbow-Wrist', 'Human', 'Robot');




