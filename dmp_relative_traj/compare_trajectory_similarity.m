%% This script compare the results of Pure IK, Hujin's method(affine deformation) and ours by calculating Frechet distances for each trajectory.


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
pos_scaling_file_name = '../motion-retargeting/mocap_ik_results_YuMi_pos_scaling.h5';


% group_name_list = {'baozhu_1', 'gun_2', 'fengren_1', 'kaoqin_2', ...
%                    'minzheng_1', 'kai_3', 'juan_2', 'jidong_1', ...
%                    'chengjian_1', 'jieshou_7', 'pao_3', 'qiao_2', ...
%                    'qie_6', 'shuan_1', 'zhenli_9'}; % list of names of motion that need processing
group_name_list = {'baozhu_1', 'gun_2', 'fengren_1'};
               
num_groups = size(group_name_list, 2);

num_datapoints = 50;

debug = false; % display of frdist() function


%% Compute Frechet distances of each trajectory pairs for each selected motion listed in group_name_list
% Initialization
% 1 - Pure IK
% absolute
frdist_l_wrist_pos_pure_ik = zeros(1, num_groups);
frdist_l_elbow_pos_pure_ik = zeros(1, num_groups);
frdist_r_wrist_pos_pure_ik = zeros(1, num_groups);
frdist_r_elbow_pos_pure_ik = zeros(1, num_groups);
% relative
frdist_lrw_pos_pure_ik = zeros(1, num_groups);
frdist_lew_pos_pure_ik = zeros(1, num_groups);
frdist_rew_pos_pure_ik = zeros(1, num_groups);
% orientation
frdist_l_wrist_eul_pure_ik = zeros(1, num_groups);
frdist_r_wrist_eul_pure_ik = zeros(1, num_groups);

% 2 - Hujin's method (affine deformation based)
% absolute
frdist_l_wrist_pos_hujin = zeros(1, num_groups);
frdist_l_elbow_pos_hujin = zeros(1, num_groups);
frdist_r_wrist_pos_hujin = zeros(1, num_groups);
frdist_r_elbow_pos_hujin = zeros(1, num_groups);
% relative
frdist_lrw_pos_hujin = zeros(1, num_groups);
frdist_lew_pos_hujin = zeros(1, num_groups);
frdist_rew_pos_hujin = zeros(1, num_groups);
% orientation
frdist_l_wrist_eul_hujin = zeros(1, num_groups);
frdist_r_wrist_eul_hujin = zeros(1, num_groups);

% 3 - Ours (DMP based)
% absolute
frdist_l_wrist_pos_ours = zeros(1, num_groups);
frdist_l_elbow_pos_ours = zeros(1, num_groups);
frdist_r_wrist_pos_ours = zeros(1, num_groups);
frdist_r_elbow_pos_ours = zeros(1, num_groups);
% relative
frdist_lrw_pos_ours = zeros(1, num_groups);
frdist_lew_pos_ours = zeros(1, num_groups);
frdist_rew_pos_ours = zeros(1, num_groups);
% orientation
frdist_l_wrist_eul_ours = zeros(1, num_groups);
frdist_r_wrist_eul_ours = zeros(1, num_groups);

% 4 - Position Scaling (w.r.t the center of shoulders)
% absolute
frdist_l_wrist_pos_pos_scaling = zeros(1, num_groups);
frdist_l_elbow_pos_pos_scaling = zeros(1, num_groups);
frdist_r_wrist_pos_pos_scaling = zeros(1, num_groups);
frdist_r_elbow_pos_pos_scaling = zeros(1, num_groups);
% relative
frdist_lrw_pos_pos_scaling  = zeros(1, num_groups);
frdist_lew_pos_pos_scaling  = zeros(1, num_groups);
frdist_rew_pos_pos_scaling  = zeros(1, num_groups);
% orientation
frdist_l_wrist_eul_pos_scaling  = zeros(1, num_groups);
frdist_r_wrist_eul_pos_scaling  = zeros(1, num_groups);


for g = 1 : num_groups
    %% Get group name
    group_name = group_name_list{g};
    
    %% Load original motion data and retargeted motion
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
    % normalize to [0,1] x [0,1] x [0,1] after `relative` is computed
    for i = 1 : 3
        % absolute
        l_wrist_pos_human(i, :) = mapminmax(l_wrist_pos_human(i, :), 0, 1);
        r_wrist_pos_human(i, :) = mapminmax(r_wrist_pos_human(i, :), 0, 1);
        l_elbow_pos_human(i, :) = mapminmax(l_elbow_pos_human(i, :), 0, 1);
        r_elbow_pos_human(i, :) = mapminmax(r_elbow_pos_human(i, :), 0, 1);
    end
    % orientation, in [w, x, y, z]
    l_wrist_quat_human = h5read(ori_file_name, ['/', group_name, '/l_wrist_quat_resampled']);
    r_wrist_quat_human = h5read(ori_file_name, ['/', group_name, '/r_wrist_quat_resampled']);
    l_wrist_eul_human = quat2eul(l_wrist_quat_human', 'XYZ')';
    r_wrist_eul_human = quat2eul(r_wrist_quat_human', 'XYZ')';
    
    % 2 - Pure IK results
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
    for j = 1 : 3
        % absolute
        l_wrist_pos_pure_ik(j, :) = mapminmax(l_wrist_pos_pure_ik(j, :), 0, 1);
        r_wrist_pos_pure_ik(j, :) = mapminmax(r_wrist_pos_pure_ik(j, :), 0, 1);
        l_elbow_pos_pure_ik(j, :) = mapminmax(l_elbow_pos_pure_ik(j, :), 0, 1);
        r_elbow_pos_pure_ik(j, :) = mapminmax(r_elbow_pos_pure_ik(j, :), 0, 1);
    end
    % orientation, in [x, y, z, w]
    l_wrist_quat_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
    r_wrist_quat_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
    l_wrist_quat_pure_ik = [l_wrist_quat_pure_ik(4, :); l_wrist_quat_pure_ik(1:3, :)];
    r_wrist_quat_pure_ik = [r_wrist_quat_pure_ik(4, :); r_wrist_quat_pure_ik(1:3, :)];
    l_wrist_eul_pure_ik = quat2eul(l_wrist_quat_pure_ik', 'XYZ')';
    r_wrist_eul_pure_ik = quat2eul(r_wrist_quat_pure_ik', 'XYZ')';
    
    % 3 - Hujin's method (more precisely, affine transformation based)
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
    for j = 1 : 3
        % absolute
        actual_l_wrist_pos_traj_hujin(j, :) = mapminmax(actual_l_wrist_pos_traj_hujin(j, :), 0, 1);
        actual_r_wrist_pos_traj_hujin(j, :) = mapminmax(actual_r_wrist_pos_traj_hujin(j, :), 0, 1);
        actual_l_elbow_pos_traj_hujin(j, :) = mapminmax(actual_l_elbow_pos_traj_hujin(j, :), 0, 1);
        actual_r_elbow_pos_traj_hujin(j, :) = mapminmax(actual_r_elbow_pos_traj_hujin(j, :), 0, 1);
    end
    % orientation, in [x, y, z, w]
    l_wrist_quat_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
    r_wrist_quat_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
    l_wrist_quat_hujin = [l_wrist_quat_hujin(4, :); l_wrist_quat_hujin(1:3, :)];
    r_wrist_quat_hujin = [r_wrist_quat_hujin(4, :); r_wrist_quat_hujin(1:3, :)];
    l_wrist_eul_hujin = quat2eul(l_wrist_quat_hujin', 'XYZ')';
    r_wrist_eul_hujin = quat2eul(r_wrist_quat_hujin', 'XYZ')';

    % 4 - Ours (DMP-based motion retargeting)
    best_round = h5read(our_file_name, ['/', group_name, '/best_round']);
    % absolute
    actual_l_wrist_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_l_wrist_pos_traj_', num2str(best_round)]);
    actual_r_wrist_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_r_wrist_pos_traj_', num2str(best_round)]);
    actual_l_elbow_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_l_elbow_pos_traj_', num2str(best_round)]);
    actual_r_elbow_pos_traj = h5read(our_file_name, ['/', group_name, '/actual_r_elbow_pos_traj_', num2str(best_round)]);
    % relative
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
    end
    % orientation, in [x, y, z, w]
    l_wrist_quat_our = h5read(our_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
    r_wrist_quat_our = h5read(our_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
    l_wrist_quat_our = [l_wrist_quat_our(4, :); l_wrist_quat_our(1:3, :)];
    r_wrist_quat_our = [r_wrist_quat_our(4, :); r_wrist_quat_our(1:3, :)];
    l_wrist_eul_our = quat2eul(l_wrist_quat_our', 'XYZ')';
    r_wrist_eul_our = quat2eul(r_wrist_quat_our', 'XYZ')';
    
    % 5 - Position scaling method
    % absolute
    actual_l_wrist_pos_traj_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_l_wrist_pos']);
    actual_r_wrist_pos_traj_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_r_wrist_pos']);
    actual_l_elbow_pos_traj_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_l_elbow_pos']);
    actual_r_elbow_pos_traj_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_r_elbow_pos']);
    % relative
    actual_lrw_pos_traj_pos_scaling = actual_l_wrist_pos_traj_pos_scaling - actual_r_wrist_pos_traj_pos_scaling;
    actual_lew_pos_traj_pos_scaling = actual_l_elbow_pos_traj_pos_scaling - actual_l_wrist_pos_traj_pos_scaling;
    actual_rew_pos_traj_pos_scaling = actual_r_elbow_pos_traj_pos_scaling - actual_r_wrist_pos_traj_pos_scaling;
    % normalize (absolute)
    for j = 1 : 3
        % absolute
        actual_l_wrist_pos_traj_pos_scaling(j, :) = mapminmax(actual_l_wrist_pos_traj_pos_scaling(j, :), 0, 1);
        actual_r_wrist_pos_traj_pos_scaling(j, :) = mapminmax(actual_r_wrist_pos_traj_pos_scaling(j, :), 0, 1);
        actual_l_elbow_pos_traj_pos_scaling(j, :) = mapminmax(actual_l_elbow_pos_traj_pos_scaling(j, :), 0, 1);
        actual_r_elbow_pos_traj_pos_scaling(j, :) = mapminmax(actual_r_elbow_pos_traj_pos_scaling(j, :), 0, 1);
    end
    % orientation, in [x, y, z, w]
    l_wrist_quat_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_l_wrist_ori_traj']);
    r_wrist_quat_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_r_wrist_ori_traj']);
    l_wrist_quat_pos_scaling = [l_wrist_quat_pos_scaling(4, :); l_wrist_quat_pos_scaling(1:3, :)];
    r_wrist_quat_pos_scaling = [r_wrist_quat_pos_scaling(4, :); r_wrist_quat_pos_scaling(1:3, :)];
    l_wrist_eul_pos_scaling = quat2eul(l_wrist_quat_pos_scaling', 'XYZ')';
    r_wrist_eul_pos_scaling = quat2eul(r_wrist_quat_pos_scaling', 'XYZ')';
    
    
    %% Calculate Frechet distances
    % 1 - Pure IK
    % absolute
    frdist_l_wrist_pos_pure_ik(g) = frdist(l_wrist_pos_human, l_wrist_pos_pure_ik, debug);
    frdist_l_elbow_pos_pure_ik(g) = frdist(l_elbow_pos_human, l_elbow_pos_pure_ik, debug);
    frdist_r_wrist_pos_pure_ik(g) = frdist(r_wrist_pos_human, r_wrist_pos_pure_ik, debug);
    frdist_r_elbow_pos_pure_ik(g) = frdist(r_elbow_pos_human, r_elbow_pos_pure_ik, debug);
    % relative
    frdist_lrw_pos_pure_ik(g) = frdist(lrw_pos_human, lrw_pos_pure_ik, debug);
    frdist_lew_pos_pure_ik(g) = frdist(lew_pos_human, lew_pos_pure_ik, debug);
    frdist_rew_pos_pure_ik(g) = frdist(rew_pos_human, rew_pos_pure_ik, debug);
    % orientation
    frdist_l_wrist_eul_pure_ik(g) = frdist(l_wrist_eul_human, l_wrist_eul_pure_ik, debug);
    frdist_r_wrist_eul_pure_ik(g) = frdist(r_wrist_eul_human, r_wrist_eul_pure_ik, debug);
    
    % 2 - Hujin's method (affine deformation based)
    % absolute
    frdist_l_wrist_pos_hujin(g) = frdist(l_wrist_pos_human, actual_l_wrist_pos_traj_hujin, debug);
    frdist_l_elbow_pos_hujin(g) = frdist(l_elbow_pos_human, actual_l_elbow_pos_traj_hujin, debug);
    frdist_r_wrist_pos_hujin(g) = frdist(r_wrist_pos_human, actual_r_wrist_pos_traj_hujin, debug);
    frdist_r_elbow_pos_hujin(g) = frdist(r_elbow_pos_human, actual_r_elbow_pos_traj_hujin, debug);
    % relative
    frdist_lrw_pos_hujin(g) = frdist(lrw_pos_human, actual_lrw_pos_traj_hujin, debug);
    frdist_lew_pos_hujin(g) = frdist(lew_pos_human, actual_lew_pos_traj_hujin, debug);
    frdist_rew_pos_hujin(g) = frdist(rew_pos_human, actual_rew_pos_traj_hujin, debug);
    % orientation
    frdist_l_wrist_eul_hujin(g) = frdist(l_wrist_eul_human, l_wrist_eul_hujin, debug);
    frdist_r_wrist_eul_hujin(g) = frdist(r_wrist_eul_human, r_wrist_eul_hujin, debug);
    
    % 3 - Ours (DMP based)
    % absolute
    frdist_l_wrist_pos_ours(g) = frdist(l_wrist_pos_human, actual_l_wrist_pos_traj, debug);
    frdist_l_elbow_pos_ours(g) = frdist(l_elbow_pos_human, actual_l_elbow_pos_traj, debug);
    frdist_r_wrist_pos_ours(g) = frdist(r_wrist_pos_human, actual_r_wrist_pos_traj, debug);
    frdist_r_elbow_pos_ours(g) = frdist(r_elbow_pos_human, actual_r_elbow_pos_traj, debug);
    % relative
    frdist_lrw_pos_ours(g) = frdist(lrw_pos_human, actual_lrw_pos_traj, debug);
    frdist_lew_pos_ours(g) = frdist(lew_pos_human, actual_lew_pos_traj, debug);
    frdist_rew_pos_ours(g) = frdist(rew_pos_human, actual_rew_pos_traj, debug);
    % orientation
    frdist_l_wrist_eul_ours(g) = frdist(l_wrist_eul_human, l_wrist_eul_our, debug);
    frdist_r_wrist_eul_ours(g) = frdist(r_wrist_eul_human, r_wrist_eul_our, debug);
    
    % 3 - Ours (DMP based)
    % absolute
    frdist_l_wrist_pos_pos_scaling(g) = frdist(l_wrist_pos_human, actual_l_wrist_pos_traj_pos_scaling, debug);
    frdist_l_elbow_pos_pos_scaling(g) = frdist(l_elbow_pos_human, actual_l_elbow_pos_traj_pos_scaling, debug);
    frdist_r_wrist_pos_pos_scaling(g) = frdist(r_wrist_pos_human, actual_r_wrist_pos_traj_pos_scaling, debug);
    frdist_r_elbow_pos_pos_scaling(g) = frdist(r_elbow_pos_human, actual_r_elbow_pos_traj_pos_scaling, debug);
    % relative
    frdist_lrw_pos_pos_scaling(g) = frdist(lrw_pos_human, actual_lrw_pos_traj_pos_scaling, debug);
    frdist_lew_pos_pos_scaling(g) = frdist(lew_pos_human, actual_lew_pos_traj_pos_scaling, debug);
    frdist_rew_pos_pos_scaling(g) = frdist(rew_pos_human, actual_rew_pos_traj_pos_scaling, debug);
    % orientation
    frdist_l_wrist_eul_pos_scaling(g) = frdist(l_wrist_eul_human, l_wrist_eul_pos_scaling, debug);
    frdist_r_wrist_eul_pos_scaling(g) = frdist(r_wrist_eul_human, r_wrist_eul_pos_scaling, debug);
    
    
    %% Display results
    disp(['>>>> ', group_name, ' <<<<']);
    % 1 - Pure IK
    disp('>> 1. Pure IK');
    msg_pure_ik = sprintf(['F_lw=%0.5f, \t F_rw=%0.5f, ' ...
                         '\t F_le=%0.5f, \t F_re=%0.5f, ' ...
                         '\t F_lrw=%0.5f, \t F_lew=%0.5f, \t F_rew=%0.5f, ' ...
                         '\t F_lw_eul=%0.5f, \t F_rw_eul=%0.5f'], ...
                         frdist_l_wrist_pos_pure_ik(g), frdist_r_wrist_pos_pure_ik(g), ...
                         frdist_l_elbow_pos_pure_ik(g), frdist_r_elbow_pos_pure_ik(g), ...
                         frdist_lrw_pos_pure_ik(g), frdist_lew_pos_pure_ik(g), frdist_rew_pos_pure_ik(g), ...
                         frdist_l_wrist_eul_pure_ik(g), frdist_r_wrist_eul_pure_ik(g));
    disp(msg_pure_ik);
    % 2 - Hujin's method
    disp('>> 2. Affine Transformation');
    msg_hujin = sprintf(['F_lw=%0.5f, \t F_rw=%0.5f, ' ...
                         '\t F_le=%0.5f, \t F_re=%0.5f, ' ...
                         '\t F_lrw=%0.5f, \t F_lew=%0.5f, \t F_rew=%0.5f, '...
                         '\t F_lw_eul=%0.5f, \t F_rw_eul=%0.5f'], ...
                         frdist_l_wrist_pos_hujin(g), frdist_r_wrist_pos_hujin(g), ...
                         frdist_l_elbow_pos_hujin(g), frdist_r_elbow_pos_hujin(g), ...
                         frdist_lrw_pos_hujin(g), frdist_lew_pos_hujin(g), frdist_rew_pos_hujin(g), ...
                         frdist_l_wrist_eul_hujin(g), frdist_r_wrist_eul_hujin(g));
    disp(msg_hujin);
    % 3 - Ours
    disp('>> 3. Ours');
    msg_ours = sprintf(['F_lw=%0.5f, \t F_rw=%0.5f, ' ...
                         '\t F_le=%0.5f, \t F_re=%0.5f, ' ...
                         '\t F_lrw=%0.5f, \t F_lew=%0.5f, \t F_rew=%0.5f, '...
                         '\t F_lw_eul=%0.5f, \t F_rw_eul=%0.5f'], ...
                         frdist_l_wrist_pos_ours(g), frdist_r_wrist_pos_ours(g), ...
                         frdist_l_elbow_pos_ours(g), frdist_r_elbow_pos_ours(g), ...
                         frdist_lrw_pos_ours(g), frdist_lew_pos_ours(g), frdist_rew_pos_ours(g), ...
                         frdist_l_wrist_eul_ours(g), frdist_r_wrist_eul_ours(g));
    disp(msg_ours);
    % 3 - Position Scaling
    disp('>> 3. Position Scaling');
    msg_pos_scaling = sprintf(['F_lw=%0.5f, \t F_rw=%0.5f, ' ...
                         '\t F_le=%0.5f, \t F_re=%0.5f, ' ...
                         '\t F_lrw=%0.5f, \t F_lew=%0.5f, \t F_rew=%0.5f, '...
                         '\t F_lw_eul=%0.5f, \t F_rw_eul=%0.5f'], ...
                         frdist_l_wrist_pos_pos_scaling(g), frdist_r_wrist_pos_pos_scaling(g), ...
                         frdist_l_elbow_pos_pos_scaling(g), frdist_r_elbow_pos_pos_scaling(g), ...
                         frdist_lrw_pos_pos_scaling(g), frdist_lew_pos_pos_scaling(g), frdist_rew_pos_pos_scaling(g), ...
                         frdist_l_wrist_eul_pos_scaling(g), frdist_r_wrist_eul_pos_scaling(g));
    disp(msg_pos_scaling);
    
end    


