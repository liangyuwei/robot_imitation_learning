%% compare Hujin, pure IK and ours



%% Prep
clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj
addpath('./'); % add all to include class folders

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
hujin_file_name = '../motion-retargeting/mocap_ik_results_YuMi_hujin.h5';
our_file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
pure_ik_file_name = '../motion-retargeting/test_imi_data_YuMi_pure_ik.h5';

group_name = 'fengren_1';%'kai_2';%'baozhu_1';%'fengren_1';

num_datapoints = 50;

max_round = h5read(our_file_name, ['/', group_name, '/max_round']); % for ease of display
best_round = h5read(our_file_name, ['/', group_name, '/best_round']); % for ease of display


% if normalize original and actually tracked trajectories
normalize_flag = true; % this is a must, because human stands in a different place in space 

% store pics 
if (normalize_flag)
    store_pics_folder = ['/home/liangyuwei/pics/2020-10-15/', group_name, '_normalized/'];
else
    store_pics_folder = ['/home/liangyuwei/pics/2020-10-15/', group_name, '/'];
end    
mkdir(store_pics_folder);


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
if (normalize_flag)
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
end
% orientation, in [w, x, y, z]
l_wrist_quat_human = h5read(ori_file_name, ['/', group_name, '/l_wrist_quat_resampled']);
r_wrist_quat_human = h5read(ori_file_name, ['/', group_name, '/r_wrist_quat_resampled']);
l_wrist_eul_human = quat2eul(l_wrist_quat_human', 'XYZ')';
r_wrist_eul_human = quat2eul(r_wrist_quat_human', 'XYZ')';
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


%% Calculate Frechet distance
%{
% max_round = 0; % for checking frechet distance using generalized DMP new trajs
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
%
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
    if (normalize_flag)
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
%}


% used with DMP_learn to check the performance of Frechet distance
%{
y_lw = y_rw + y_lrw;
y_le = y_lw + y_lew;
y_re = y_rw + y_rew;
% iterate over optimization results of different rounds
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        y_lw(j, :) = mapminmax(y_lw(j, :), 0, 1);
        y_rw(j, :) = mapminmax(y_rw(j, :), 0, 1);
        y_le(j, :) = mapminmax(y_le(j, :), 0, 1);
        y_re(j, :) = mapminmax(y_re(j, :), 0, 1);
        % relative
    %         y_lrw(j, :) = mapminmax(y_lrw(j, :), 0, 1);
    %         y_lew(j, :) = mapminmax(y_lew(j, :), 0, 1);
    %         y_rew(j, :) = mapminmax(y_rew(j, :), 0, 1);
    end
end
    
% compute discrete Frechet distance (absolute)
frdist_l_wrist_pos(1) = frdist(l_wrist_pos_human, y_lw);
frdist_l_elbow_pos(1) = frdist(l_elbow_pos_human, y_le);
frdist_r_wrist_pos(1) = frdist(r_wrist_pos_human, y_rw);
frdist_r_elbow_pos(1) = frdist(r_elbow_pos_human, y_re);
% compute discrete Frechet distance (relative)
frdist_lrw_pos(1) = frdist(lrw_pos_human, y_lrw);    
frdist_lew_pos(1) = frdist(lew_pos_human, y_lew);    
frdist_rew_pos(1) = frdist(rew_pos_human, y_rew);    

% align original data for display
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

% plot the result
plot_3d_subplots(l_wrist_pos_human_aligned, y_lw, {'Left Wrist', ['Frechet Distance = ', num2str(frdist_l_wrist_pos)]}, 'Human', 'Robot');
plot_3d_subplots(r_wrist_pos_human_aligned, y_rw, {'Right Wrist', ['Frechet Distance = ', num2str(frdist_r_wrist_pos)]}, 'Human', 'Robot');
plot_3d_subplots(l_elbow_pos_human_aligned, y_le, {'Left Elbow', ['Frechet Distance = ', num2str(frdist_l_elbow_pos)]}, 'Human', 'Robot');
plot_3d_subplots(r_elbow_pos_human_aligned, y_re, {'Right Elbow', ['Frechet Distance = ', num2str(frdist_r_elbow_pos)]}, 'Human', 'Robot');

plot_3d_subplots(lrw_pos_human_aligned, y_lrw, {'Left-Right Wrist', ['Frechet Distance = ', num2str(frdist_lrw_pos)]}, 'Human', 'Robot');
plot_3d_subplots(lew_pos_human_aligned, y_lew, {'Left Elbow-Wrist', ['Frechet Distance = ', num2str(frdist_lew_pos)]}, 'Human', 'Robot');
plot_3d_subplots(rew_pos_human_aligned, y_rew, {'Right Elbow-Wrist', ['Frechet Distance = ', num2str(frdist_rew_pos)]}, 'Human', 'Robot');

%}


%% Display all Frechet distance history in one figure
%{
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


%% Load motion retargeting results
% 1 - our method: the best tracked round's result(may not be max_round); actually tracked trajectories     
% read actually tracked trajectories (absolute)
actual_l_wrist_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_l_wrist_pos_traj_', num2str(best_round)]);
actual_r_wrist_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_r_wrist_pos_traj_', num2str(best_round)]);
actual_l_elbow_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_l_elbow_pos_traj_', num2str(best_round)]);
actual_r_elbow_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_r_elbow_pos_traj_', num2str(best_round)]);
% relative
actual_lrw_pos_traj = actual_l_wrist_pos_traj - actual_r_wrist_pos_traj;
actual_lew_pos_traj = actual_l_elbow_pos_traj - actual_l_wrist_pos_traj;
actual_rew_pos_traj = actual_r_elbow_pos_traj - actual_r_wrist_pos_traj;
% normalize (absolute)
if (normalize_flag)
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
end
% orientation, in [x, y, z, w]
l_wrist_quat_our = h5read(our_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
r_wrist_quat_our = h5read(our_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
l_wrist_quat_our = [l_wrist_quat_our(4, :); l_wrist_quat_our(1:3, :)];
r_wrist_quat_our = [r_wrist_quat_our(4, :); r_wrist_quat_our(1:3, :)];
l_wrist_eul_our = quat2eul(l_wrist_quat_our', 'XYZ')';
r_wrist_eul_our = quat2eul(r_wrist_quat_our', 'XYZ')';


% 2 - Hujin's method
% absolute
actual_l_wrist_pos_traj_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_l_wrist_pos_traj_hujin']);
actual_r_wrist_pos_traj_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_r_wrist_pos_traj_hujin']);
actual_l_elbow_pos_traj_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_l_elbow_pos_traj_hujin']);
actual_r_elbow_pos_traj_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_r_elbow_pos_traj_hujin']);
% relative
actual_lrw_pos_traj_hujin = actual_l_wrist_pos_traj_hujin - actual_r_wrist_pos_traj_hujin;
actual_lew_pos_traj_hujin = actual_l_elbow_pos_traj_hujin - actual_l_wrist_pos_traj_hujin;
actual_rew_pos_traj_hujin = actual_r_elbow_pos_traj_hujin - actual_r_wrist_pos_traj_hujin;
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        actual_l_wrist_pos_traj_hujin(j, :) = mapminmax(actual_l_wrist_pos_traj_hujin(j, :), 0, 1);
        actual_r_wrist_pos_traj_hujin(j, :) = mapminmax(actual_r_wrist_pos_traj_hujin(j, :), 0, 1);
        actual_l_elbow_pos_traj_hujin(j, :) = mapminmax(actual_l_elbow_pos_traj_hujin(j, :), 0, 1);
        actual_r_elbow_pos_traj_hujin(j, :) = mapminmax(actual_r_elbow_pos_traj_hujin(j, :), 0, 1);
        % relative
%         actual_lrw_pos_traj(j, :) = mapminmax(actual_lrw_pos_traj(j, :), 0, 1);
%         actual_lew_pos_traj(j, :) = mapminmax(actual_lew_pos_traj(j, :), 0, 1);
%         actual_rew_pos_traj(j, :) = mapminmax(actual_rew_pos_traj(j, :), 0, 1);
    end
end
% orientation, in [x, y, z, w]
l_wrist_quat_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
r_wrist_quat_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
l_wrist_quat_hujin = [l_wrist_quat_hujin(4, :); l_wrist_quat_hujin(1:3, :)];
r_wrist_quat_hujin = [r_wrist_quat_hujin(4, :); r_wrist_quat_hujin(1:3, :)];
l_wrist_eul_hujin = quat2eul(l_wrist_quat_hujin', 'XYZ')';
r_wrist_eul_hujin = quat2eul(r_wrist_quat_hujin', 'XYZ')';
% align the length
actual_l_wrist_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_l_elbow_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_r_wrist_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_r_elbow_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_lrw_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_lew_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_rew_pos_traj_hujin_aligned = zeros(3, num_datapoints);
actual_l_wrist_eul_traj_hujin_aligned = zeros(3, num_datapoints);
actual_r_wrist_eul_traj_hujin_aligned = zeros(3, num_datapoints);
original_num_datapoints = size(actual_l_wrist_pos_traj_hujin, 2);
original_timestamps = linspace(0, 1, original_num_datapoints);
target_timestamps = linspace(0, 1, num_datapoints);
for i = 1 : 3
    actual_l_wrist_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_l_wrist_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_l_elbow_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_l_elbow_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_r_wrist_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_r_wrist_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_r_elbow_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_r_elbow_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_lrw_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_lrw_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_lew_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_lew_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_rew_pos_traj_hujin_aligned(i, :) = interp1(original_timestamps, actual_rew_pos_traj_hujin(i, :), target_timestamps, 'linear');
    actual_l_wrist_eul_traj_hujin_aligned(i, :) = interp1(original_timestamps, l_wrist_eul_hujin(i, :), target_timestamps, 'linear');    
    actual_r_wrist_eul_traj_hujin_aligned(i, :) = interp1(original_timestamps, r_wrist_eul_hujin(i, :), target_timestamps, 'linear');    
end


% 3 - Pure IK results
% absolute
l_wrist_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/l_wrist_pos']);
r_wrist_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/r_wrist_pos']);
l_elbow_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/l_elbow_pos']);
r_elbow_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/r_elbow_pos']);
% relative
lrw_pos_pure_ik = l_wrist_pos_pure_ik - r_wrist_pos_pure_ik;
lew_pos_pure_ik = l_elbow_pos_pure_ik - l_wrist_pos_pure_ik;
rew_pos_pure_ik = r_elbow_pos_pure_ik - r_wrist_pos_pure_ik;
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        l_wrist_pos_pure_ik(j, :) = mapminmax(l_wrist_pos_pure_ik(j, :), 0, 1);
        r_wrist_pos_pure_ik(j, :) = mapminmax(r_wrist_pos_pure_ik(j, :), 0, 1);
        l_elbow_pos_pure_ik(j, :) = mapminmax(l_elbow_pos_pure_ik(j, :), 0, 1);
        r_elbow_pos_pure_ik(j, :) = mapminmax(r_elbow_pos_pure_ik(j, :), 0, 1);
        % relative
%         actual_lrw_pos_traj(j, :) = mapminmax(actual_lrw_pos_traj(j, :), 0, 1);
%         actual_lew_pos_traj(j, :) = mapminmax(actual_lew_pos_traj(j, :), 0, 1);
%         actual_rew_pos_traj(j, :) = mapminmax(actual_rew_pos_traj(j, :), 0, 1);
    end
end
% orientation, in [x, y, z, w]
l_wrist_quat_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
r_wrist_quat_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
l_wrist_quat_pure_ik = [l_wrist_quat_pure_ik(4, :); l_wrist_quat_pure_ik(1:3, :)];
r_wrist_quat_pure_ik = [r_wrist_quat_pure_ik(4, :); r_wrist_quat_pure_ik(1:3, :)];
l_wrist_eul_pure_ik = quat2eul(l_wrist_quat_pure_ik', 'XYZ')';
r_wrist_eul_pure_ik = quat2eul(r_wrist_quat_pure_ik', 'XYZ')';


%% Display the comparison results
print_flag = true;
img_wid = 16;
img_hgt = 9;
dpi = 600; % 600 dpi ? (does EPS file format has the concept???)
use_legend = true;
human_tag = '  Human';
ours_tag = '  Ours'; 
hujin_tag = '  Affine'; % tags used in legend
% absolute
fig = plot_3d_subplots_4traj(l_wrist_pos_human_aligned, actual_l_wrist_pos_traj, actual_l_wrist_pos_traj_hujin_aligned, l_wrist_pos_pure_ik, ...
    'Left Wrist Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    % save as PNG image
    saveas(gcf, [store_pics_folder, 'lw_human_robot_comp-', group_name], 'png');
    % save as EPS image, with 16:9 image size and 600 dpi resolution
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'lw_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]); % 600 dpi
end

fig = plot_3d_subplots_4traj(l_elbow_pos_human_aligned, actual_l_elbow_pos_traj, actual_l_elbow_pos_traj_hujin_aligned, l_elbow_pos_pure_ik, ...
    'Left Elbow Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'le_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'le_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

fig = plot_3d_subplots_4traj(r_wrist_pos_human_aligned, actual_r_wrist_pos_traj, actual_r_wrist_pos_traj_hujin_aligned, r_wrist_pos_pure_ik, ...
    'Right Wrist Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'rw_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'rw_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

fig = plot_3d_subplots_4traj(r_elbow_pos_human_aligned, actual_r_elbow_pos_traj, actual_r_elbow_pos_traj_hujin_aligned, r_elbow_pos_pure_ik, ...
    'Right Elbow Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 're_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 're_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

% relative
fig = plot_3d_subplots_4traj(lrw_pos_human_aligned, actual_lrw_pos_traj, actual_lrw_pos_traj_hujin_aligned, lrw_pos_pure_ik, ...
    'Left-Right Wrist Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'lrw_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'lrw_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

fig = plot_3d_subplots_4traj(lew_pos_human_aligned, actual_lew_pos_traj, actual_lew_pos_traj_hujin_aligned, lew_pos_pure_ik, ...
    'Left Elbow-Wrist Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'lew_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'lew_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

fig = plot_3d_subplots_4traj(rew_pos_human_aligned, actual_rew_pos_traj, actual_rew_pos_traj_hujin_aligned, rew_pos_pure_ik, ...
    'Right Elbow-Wrist Pos', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'rew_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'rew_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

% orientation
fig = plot_3d_subplots_4traj(l_wrist_eul_human, l_wrist_eul_our, actual_l_wrist_eul_traj_hujin_aligned, l_wrist_eul_pure_ik, ...
    'Left Wrist Eul', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'lw_eul_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'lw_eul_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end

fig = plot_3d_subplots_4traj(r_wrist_eul_human, r_wrist_eul_our, actual_r_wrist_eul_traj_hujin_aligned, r_wrist_eul_pure_ik, ...
    'Right Wrist Eul', human_tag, ours_tag, hujin_tag, 'Pure IK', use_legend);
if (print_flag)
    saveas(gcf, [store_pics_folder, 'rw_eul_human_robot_comp-', group_name], 'png');
    set(fig, 'PaperPositionMode', 'manual');
    set(fig, 'PaperPosition', [0, 0, img_wid, img_hgt]);
    print(fig, [store_pics_folder, 'rw_eul_human_robot_comp-', group_name], '-depsc', ['-r',num2str(dpi)]);
end




