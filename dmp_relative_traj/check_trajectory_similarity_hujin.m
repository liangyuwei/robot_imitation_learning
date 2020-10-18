%% Visualize the actually tracked trajectories of Hujin's method.



%% Prep
clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj
addpath('./'); % add all to include class folders

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
file_name = '../motion-retargeting/mocap_ik_results_YuMi_hujin.h5';
our_method_file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';

group_name = 'shuan_1';%'kai_2';%'baozhu_1';%'fengren_1';

num_datapoints = 50;

% if normalize original and actually tracked trajectories
normalize_flag = true;

% store pics 
if (normalize_flag)
    store_pics_folder = ['/home/liangyuwei/pics/2020-09-13/', group_name, '_normalized/'];
else
    store_pics_folder = ['/home/liangyuwei/pics/2020-09-13/', group_name, '/'];
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

% 2 - Hujin's retargeting method results
% absolute
actual_l_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_hujin']);
actual_r_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_hujin']);
actual_l_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_hujin']);
actual_r_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_hujin']);
% actual_l_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_wrist_pos_traj_hujin_original']);
% actual_r_wrist_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_wrist_pos_traj_hujin_original']);
% actual_l_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_l_elbow_pos_traj_hujin_original']);
% actual_r_elbow_pos_traj = h5read(file_name, ['/', group_name, '/actual_r_elbow_pos_traj_hujin_original']);
% relative
actual_lrw_pos_traj = actual_l_wrist_pos_traj - actual_r_wrist_pos_traj;
actual_lew_pos_traj = actual_l_elbow_pos_traj - actual_l_wrist_pos_traj;
actual_rew_pos_traj = actual_r_elbow_pos_traj - actual_r_wrist_pos_traj;


% 3 - Reference trajectories set for Hujin's method (same as what we do)
% absolute
ref_l_wrist_pos_traj = h5read(file_name, ['/', group_name, '/ref_l_wrist_traj_hujin']);
ref_r_wrist_pos_traj = h5read(file_name, ['/', group_name, '/ref_r_wrist_traj_hujin']);
ref_l_elbow_pos_traj = h5read(file_name, ['/', group_name, '/ref_l_elbow_traj_hujin']);
ref_r_elbow_pos_traj = h5read(file_name, ['/', group_name, '/ref_r_elbow_traj_hujin']);
% relative
ref_lrw_pos_traj = ref_l_wrist_pos_traj - ref_r_wrist_pos_traj;
ref_lew_pos_traj = ref_l_elbow_pos_traj - ref_l_elbow_pos_traj;
ref_rew_pos_traj = ref_r_elbow_pos_traj - ref_r_wrist_pos_traj;


% 4 - DMP initial reference trajectories used in our optimization method
kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
dt = 0.04; %1/num_datapoints; %Duration of time step
% DMP coefficients and parameters
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
% Load from file
dmp_starts_goals_initial = h5read(our_method_file_name, ['/', group_name, '/dmp_starts_goals_initial']);
new_goal_lrw_initial = dmp_starts_goals_initial(1:3); new_start_lrw_initial = dmp_starts_goals_initial(4:6);
new_goal_lew_initial = dmp_starts_goals_initial(7:9); new_start_lew_initial = dmp_starts_goals_initial(10:12);
new_goal_rew_initial = dmp_starts_goals_initial(13:15); new_start_rew_initial = dmp_starts_goals_initial(16:18);
new_goal_rw_initial = dmp_starts_goals_initial(19:21); new_start_rw_initial = dmp_starts_goals_initial(22:24);
% Reproduce DMP trajectories
y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw_initial, new_start_lrw_initial, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew_initial, new_start_lew_initial, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew_initial, new_start_rew_initial, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw_initial, new_start_rw_initial, false);
% Add up to make up the reference trajectories
y_r_wrist_initial = y_rw;
y_l_wrist_initial = y_rw + y_lrw;
y_r_elbow_initial = y_rw + y_rew;
y_l_elbow_initial = y_l_wrist_initial + y_lew;


%% Compare DMP initial reference and the reference set by us when using Hujin's method
figure;
p1 = plot3(ref_l_wrist_pos_traj(1, :), ref_l_wrist_pos_traj(2, :), ref_l_wrist_pos_traj(3, :), 'b-'); hold on; grid on;
plot3(ref_r_wrist_pos_traj(1, :), ref_r_wrist_pos_traj(2, :), ref_r_wrist_pos_traj(3, :), 'b-');
plot3(ref_l_elbow_pos_traj(1, :), ref_l_elbow_pos_traj(2, :), ref_l_elbow_pos_traj(3, :), 'b-'); 
plot3(ref_r_elbow_pos_traj(1, :), ref_r_elbow_pos_traj(2, :), ref_r_elbow_pos_traj(3, :), 'b-');
p2 = plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); 
plot3(y_r_wrist_initial(1, :), y_r_wrist_initial(2, :), y_r_wrist_initial(3, :), 'r--');
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--');
title('Reference trajectories used in Hujin''s method and in our method');
legend([p1, p2], 'Affine Reference Traj', 'DMP Reference Traj', 'Location', 'NorthEastOutside');
xlabel('x'); ylabel('y'); zlabel('z');


%% Compare reference and actually tracked trajectories in 3d view
figure;
p1 = plot3(ref_l_wrist_pos_traj(1, :), ref_l_wrist_pos_traj(2, :), ref_l_wrist_pos_traj(3, :), 'b-'); hold on; grid on;
plot3(ref_r_wrist_pos_traj(1, :), ref_r_wrist_pos_traj(2, :), ref_r_wrist_pos_traj(3, :), 'b-');
plot3(ref_l_elbow_pos_traj(1, :), ref_l_elbow_pos_traj(2, :), ref_l_elbow_pos_traj(3, :), 'b-'); 
plot3(ref_r_elbow_pos_traj(1, :), ref_r_elbow_pos_traj(2, :), ref_r_elbow_pos_traj(3, :), 'b-');
p2 = plot3(actual_l_wrist_pos_traj(1, :), actual_l_wrist_pos_traj(2, :), actual_l_wrist_pos_traj(3, :), 'r--'); 
plot3(actual_r_wrist_pos_traj(1, :), actual_r_wrist_pos_traj(2, :), actual_r_wrist_pos_traj(3, :), 'r--');
plot3(actual_l_elbow_pos_traj(1, :), actual_l_elbow_pos_traj(2, :), actual_l_elbow_pos_traj(3, :), 'r--'); 
plot3(actual_r_elbow_pos_traj(1, :), actual_r_elbow_pos_traj(2, :), actual_r_elbow_pos_traj(3, :), 'r--');
title('Reference and Actually tracked trajectories(Hujin''s method)');
legend([p1, p2], 'Reference Traj', 'Actually Tracked Traj', 'Location', 'NorthEastOutside');
xlabel('x'); ylabel('y'); zlabel('z');


%% Compare original and actually tracked trajectories in 3d view
figure;
p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-');
plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-'); 
plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-');
p2 = plot3(actual_l_wrist_pos_traj(1, :), actual_l_wrist_pos_traj(2, :), actual_l_wrist_pos_traj(3, :), 'r--'); 
plot3(actual_r_wrist_pos_traj(1, :), actual_r_wrist_pos_traj(2, :), actual_r_wrist_pos_traj(3, :), 'r--');
plot3(actual_l_elbow_pos_traj(1, :), actual_l_elbow_pos_traj(2, :), actual_l_elbow_pos_traj(3, :), 'r--'); 
plot3(actual_r_elbow_pos_traj(1, :), actual_r_elbow_pos_traj(2, :), actual_r_elbow_pos_traj(3, :), 'r--');
title('Original and Actually tracked trajectories(Hujin''s method)');
legend([p1, p2], 'Original Traj', 'Actually Tracked Traj', 'Location', 'NorthEastOutside');
xlabel('x'); ylabel('y'); zlabel('z');


%% Normalization processing for comparison (normalize to [0, 1] x [0, 1] x [0, 1])
% 1 - original 
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

% 2 - actually tracked
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


%% Calculate Frechet distance
% compute discrete Frechet distance (absolute)
frdist_l_wrist_pos = frdist(l_wrist_pos_human, actual_l_wrist_pos_traj);
frdist_l_elbow_pos = frdist(l_elbow_pos_human, actual_l_elbow_pos_traj);
frdist_r_wrist_pos = frdist(r_wrist_pos_human, actual_r_wrist_pos_traj);
frdist_r_elbow_pos = frdist(r_elbow_pos_human, actual_r_elbow_pos_traj);
% compute discrete Frechet distance (relative)
frdist_lrw_pos = frdist(lrw_pos_human, actual_lrw_pos_traj);
frdist_lew_pos = frdist(lew_pos_human, actual_lew_pos_traj);
frdist_rew_pos = frdist(rew_pos_human, actual_rew_pos_traj);


%% Display the original and actually tracked trajectories in split view
% display
% absolute
plot_3d_subplots(l_wrist_pos_human, actual_l_wrist_pos_traj, ...
                 {'Left Wrist', ['Frechet Distance = ', num2str(frdist_l_wrist_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'left_wrist_human_robot_comp'], 'png');
plot_3d_subplots(l_elbow_pos_human, actual_l_elbow_pos_traj, ...
                 {'Left Elbow', ['Frechet Distance = ', num2str(frdist_l_elbow_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'left_elbow_human_robot_comp'], 'png');
plot_3d_subplots(r_wrist_pos_human, actual_r_wrist_pos_traj, ...
                 {'Right Wrist', ['Frechet Distance = ', num2str(frdist_r_wrist_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'right_wrist_human_robot_comp'], 'png');
plot_3d_subplots(r_elbow_pos_human, actual_r_elbow_pos_traj, ...
                 {'Right Elbow', ['Frechet Distance = ', num2str(frdist_l_elbow_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'right_elbow_human_robot_comp'], 'png');


% relative
plot_3d_subplots(lrw_pos_human, actual_lrw_pos_traj, ...
                 {'Left-Right Wrist', ['Frechet Distance = ', num2str(frdist_lrw_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'lrw_human_robot_comp'], 'png');
plot_3d_subplots(lew_pos_human, actual_lew_pos_traj, ...
                 {'Left Elbow-Wrist', ['Frechet Distance = ', num2str(frdist_lew_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'lew_human_robot_comp'], 'png');
plot_3d_subplots(rew_pos_human, actual_rew_pos_traj, ...
                 {'Right Elbow-Wrist', ['Frechet Distance = ', num2str(frdist_rew_pos)]}, 'Human', 'Robot');
% saveas(gcf, [store_pics_folder, 'rew_human_robot_comp'], 'png');



