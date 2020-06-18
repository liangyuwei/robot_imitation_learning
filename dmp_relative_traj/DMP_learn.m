%% This file learns and generalizes relative position trajectories using DMP, and store the learned result in .h5 file for later use.
% Targets to encode: elbow pos relative to wrist, relative wrist pos (between left and right). Three parts in total.    

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

num_datapoints = 400;%50;

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
nbStates = 64; %Number of activation functions (i.e., number of states in the GMM)
nbVar = 1; %Number of variables for the radial basis functions [s] (decay term)
nbVarPos = 3; %2; %Number of motion variables [x1,x2] 
kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
dt = 0.01; %1/num_datapoints; %Duration of time step
nbSamples = 1; % Number of samples (of the same movement)
display = true; % display the reproduction to see the performance


%% Construct traj_dataset
% relative position between left and right wrists
traj_dataset{1} = lr_wrist_pos'; % should be of size Length x DOF!!!
% 1 - Use TP-GMM to model the goal signal
% model = DMP_TPGMM(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, true); % scalability not quite well, possibily due to modeling g instead of f/x/(g-y0)
% 2 - Multiple methods, including direct inverse(best for our case), LWR, several GMR implementations, etc.      
[Mu_lrw, Sigma_lrw, Weights_lrw, sIn_lrw, Yr_lrw] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, false);
% 3 - Direct copy of nonlinear force profile f
% f_lrw = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);
% 4 - GMR reproduces trajectory; refine with LQR. 
% model = DMP_LQR(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, true); % huge error due to unknown reason...

% debug display: output trajectory of different length, start and goal
%{
xTar = traj_dataset{1}(end, :)';% + [0, -0.5, 1.0]';
xStart = traj_dataset{1}(1, :)';% + [0, 0.5, 1.0]';
repro = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, xTar, xStart, true);
figure;
plot3(traj_dataset{1}(:, 1), traj_dataset{1}(:, 2), traj_dataset{1}(:, 3), 'b.'); hold on; grid on;
plot3(traj_dataset{1}(1, 1), traj_dataset{1}(1, 2), traj_dataset{1}(1, 3), 'go');
plot3(traj_dataset{1}(end, 1), traj_dataset{1}(end, 2), traj_dataset{1}(end, 3), 'ro');
plot3(repro(1, :), repro(2, :), repro(3, :), 'r-');
plot3(repro(1, 1), repro(2, 1), repro(3, 1), 'go');
plot3(repro(1, end), repro(2, end), repro(3, end), 'ro');
title('Reproduced trajectory and the original trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
%}


% relative position between elbow and wrist (Left)
traj_dataset{1} = l_elbow_wrist_pos'; % should be of size Length x DOF!!!
[Mu_lew, Sigma_lew, Weights_lew, sIn_lew, Yr_lew] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, false);
% f_lew = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);

% relative position between elbow and wrist (Right)
traj_dataset{1} = r_elbow_wrist_pos'; % should be of size Length x DOF!!!
[Mu_rew, Sigma_rew, Weights_rew, sIn_rew, Yr_rew] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, false);
% f_rew = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);

% absolute position of Right wrist
traj_dataset{1} = r_wrist_pos'; % should be of size Length x DOF!!!
[Mu_rw, Sigma_rw, Weights_rw, sIn_rw, Yr_rw] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, false);
% f_rw = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);



%% Store the learned results
%
file_name = 'test_imi_data_YuMi.h5';
group_name = 'fengren_1';
% data about lrw
h5create(file_name, ['/', group_name, '/Mu_lrw'], size(Mu_lrw));
h5write(file_name, ['/', group_name, '/Mu_lrw'], Mu_lrw);
h5create(file_name, ['/', group_name, '/Sigma_lrw'], size(Sigma_lrw));
h5write(file_name, ['/', group_name, '/Sigma_lrw'], Sigma_lrw);
h5create(file_name, ['/', group_name, '/Weights_lrw'], size(Weights_lrw));
h5write(file_name, ['/', group_name, '/Weights_lrw'], Weights_lrw);
h5create(file_name, ['/', group_name, '/sIn_lrw'], size(sIn_lrw));
h5write(file_name, ['/', group_name, '/sIn_lrw'], sIn_lrw);
h5create(file_name, ['/', group_name, '/Yr_lrw'], size(Yr_lrw));
h5write(file_name, ['/', group_name, '/Yr_lrw'], Yr_lrw);

% data about lew
h5create(file_name, ['/', group_name, '/Mu_lew'], size(Mu_lew));
h5write(file_name, ['/', group_name, '/Mu_lew'], Mu_lew);
h5create(file_name, ['/', group_name, '/Sigma_lew'], size(Sigma_lew));
h5write(file_name, ['/', group_name, '/Sigma_lew'], Sigma_lew);
h5create(file_name, ['/', group_name, '/Weights_lew'], size(Weights_lew));
h5write(file_name, ['/', group_name, '/Weights_lew'], Weights_lew);
h5create(file_name, ['/', group_name, '/sIn_lew'], size(sIn_lew));
h5write(file_name, ['/', group_name, '/sIn_lew'], sIn_lew);
h5create(file_name, ['/', group_name, '/Yr_lew'], size(Yr_lew));
h5write(file_name, ['/', group_name, '/Yr_lew'], Yr_lew);

% data about rew
h5create(file_name, ['/', group_name, '/Mu_rew'], size(Mu_rew));
h5write(file_name, ['/', group_name, '/Mu_rew'], Mu_rew);
h5create(file_name, ['/', group_name, '/Sigma_rew'], size(Sigma_rew));
h5write(file_name, ['/', group_name, '/Sigma_rew'], Sigma_rew);
h5create(file_name, ['/', group_name, '/Weights_rew'], size(Weights_rew));
h5write(file_name, ['/', group_name, '/Weights_rew'], Weights_rew);
h5create(file_name, ['/', group_name, '/sIn_rew'], size(sIn_rew));
h5write(file_name, ['/', group_name, '/sIn_rew'], sIn_rew);
h5create(file_name, ['/', group_name, '/Yr_rew'], size(Yr_rew));
h5write(file_name, ['/', group_name, '/Yr_rew'], Yr_rew);

% data about rw
h5create(file_name, ['/', group_name, '/Mu_rw'], size(Mu_rw));
h5write(file_name, ['/', group_name, '/Mu_rw'], Mu_rw);
h5create(file_name, ['/', group_name, '/Sigma_rw'], size(Sigma_rw));
h5write(file_name, ['/', group_name, '/Sigma_rw'], Sigma_rw);
h5create(file_name, ['/', group_name, '/Weights_rw'], size(Weights_rw));
h5write(file_name, ['/', group_name, '/Weights_rw'], Weights_rw);
h5create(file_name, ['/', group_name, '/sIn_rw'], size(sIn_rw));
h5write(file_name, ['/', group_name, '/sIn_rw'], sIn_rw);
h5create(file_name, ['/', group_name, '/Yr_rw'], size(Yr_rw));
h5write(file_name, ['/', group_name, '/Yr_rw'], Yr_rw);

%}

%% Reproduce trajectory
new_goal_lrw = lr_wrist_pos(:, end) + [0.01, -0.01, 0.05]';
new_start_lrw = lr_wrist_pos(:, 1) + [-0.01, 0.01, 0.05]';
new_goal_lew = l_elbow_wrist_pos(:, end) + [0.01, 0.0, -0.05]';
new_start_lew = l_elbow_wrist_pos(:, 1) + [0.01, -0.01, -0.05]';
new_goal_rew = r_elbow_wrist_pos(:, end) + [0.01, 0.02, 0.03]';
new_start_rew = r_elbow_wrist_pos(:, 1) + [0.02, 0.01, 0.05]';
new_goal_rw = r_wrist_pos(:, end) + [-0.02, 0.0, 0.0]';
new_start_rw = r_wrist_pos(:, 1) + [-0.02, 0.0, 0.0]';
% y_lrw = DMP_use_f(f_lrw, nbData, alpha, kP, kV, new_goal_lrw, new_start_lrw);
% y_lew = DMP_use_f(f_lew, nbData, alpha, kP, kV, new_goal_lew, new_start_lew);
% y_rew = DMP_use_f(f_rew, nbData, alpha, kP, kV, new_goal_rew, new_start_rew);
% y_rw = DMP_use_f(f_rw, nbData, alpha, kP, kV, new_goal_rw, new_start_rw);

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, 50, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, 50, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, 50, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, 50, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);


%
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

