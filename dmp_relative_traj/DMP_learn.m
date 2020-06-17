%% This file learns and generalizes relative position trajectories using DMP, and store the learned result in .h5 file for later use.
% Targets to encode: elbow pos relative to wrist, relative wrist pos (between left and right). Three parts in total.    

% addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

num_datapoints = 200;%50;

%% Load demonstration
file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
group_name = 'fengren_1';

time = h5read(file_name, ['/', group_name, '/time']);
l_wrist_pos = resample_traj(time, h5read(file_name, ['/', group_name, '/l_wrist_pos']), num_datapoints, false);
l_elbow_pos = resample_traj(time, h5read(file_name, ['/', group_name, '/l_elbow_pos']), num_datapoints, false);
r_wrist_pos = resample_traj(time, h5read(file_name, ['/', group_name, '/r_wrist_pos']), num_datapoints, false);
r_elbow_pos = resample_traj(time, h5read(file_name, ['/', group_name, '/r_elbow_pos']), num_datapoints, false); % input should be DOF x length

lr_wrist_pos = l_wrist_pos - r_wrist_pos;
l_elbow_wrist_pos = l_elbow_pos - l_wrist_pos;
r_elbow_wrist_pos = r_elbow_pos - r_wrist_pos; 


%% DMP settings
nbData = num_datapoints; % resampling inside DMP_learn.m !!!
nbStates = 96; %Number of activation functions (i.e., number of states in the GMM)
nbVar = 1; %Number of variables for the radial basis functions [s] (decay term)
nbVarPos = 3; %2; %Number of motion variables [x1,x2] 
kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
% dt = 0.01; %1/num_datapoints; %Duration of time step
nbSamples = 1; % Number of samples (of the same movement)
display = true; % display the reproduction to see the performance


%% Construct traj_dataset
% relative position between left and right wrists
traj_dataset{1} = lr_wrist_pos'; % should be of size Length x DOF!!!
f_lrw = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, true);
% f_lrw = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);
         

% relative position between elbow and wrist (Left)
traj_dataset{1} = l_elbow_wrist_pos'; % should be of size Length x DOF!!!
f_lew = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);

% relative position between elbow and wrist (Right)
traj_dataset{1} = r_elbow_wrist_pos'; % should be of size Length x DOF!!!
f_rew = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);

% absolute position of Right wrist
traj_dataset{1} = r_wrist_pos'; % should be of size Length x DOF!!!
f_rw = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);


%% Reproduce trajectory
new_goal_lrw = lr_wrist_pos(:, end) + [0.05, -0.05, 0.05]';
new_start_lrw = lr_wrist_pos(:, 1) + [-0.05, 0.05, 0.05]';
new_goal_lew = l_elbow_wrist_pos(:, end) + [0.05, 0.0, -0.05]';
new_start_lew = l_elbow_wrist_pos(:, 1) + [0.0, -0.05, -0.05]';
new_goal_rew = r_elbow_wrist_pos(:, end) + [0.01, 0.08, 0.03]';
new_start_rew = r_elbow_wrist_pos(:, 1) + [0.02, 0.1, 0.05]';
new_goal_rw = r_wrist_pos(:, end) + [-0.05, 0.0, 0.0]';
new_start_rw = r_wrist_pos(:, 1) + [-0.05, 0.0, 0.0]';
y_lrw = DMP_use_f(f_lrw, nbData, alpha, kP, kV, new_goal_lrw, new_start_lrw);
y_lew = DMP_use_f(f_lew, nbData, alpha, kP, kV, new_goal_lew, new_start_lew);
y_rew = DMP_use_f(f_rew, nbData, alpha, kP, kV, new_goal_rew, new_start_rew);
y_rw = DMP_use_f(f_rw, nbData, alpha, kP, kV, new_goal_rw, new_start_rw);

%{
figure; plot3(y_rw(1,:), y_rw(2,:), y_rw(3,:), 'r.'); hold on; grid on;
plot3(r_wrist_pos(1,:), r_wrist_pos(2,:), r_wrist_pos(3,:), 'b.');
title('r\_wist\_pos');

figure; plot3(y_lrw(1,:), y_lrw(2,:), y_lrw(3,:), 'r.'); hold on; grid on;
plot3(lr_wrist_pos(1,:), lr_wrist_pos(2,:), lr_wrist_pos(3,:), 'b.');
title('lr\_wrist\_pos');

figure; plot3(y_lew(1,:), y_lew(2,:), y_lew(3,:), 'r.'); hold on; grid on;
plot3(l_elbow_wrist_pos(1,:), l_elbow_wrist_pos(2,:), l_elbow_wrist_pos(3,:), 'b.');
title('l\_elbow\_wrist\_pos');

figure; plot3(y_rew(1,:), y_rew(2,:), y_rew(3,:), 'r.'); hold on; grid on;
plot3(r_elbow_wrist_pos(1,:), r_elbow_wrist_pos(2,:), r_elbow_wrist_pos(3,:), 'b.');
title('r\_elbow\_wrist\_pos');
%}


%% display the results
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

