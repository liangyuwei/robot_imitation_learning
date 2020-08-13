%% This file learns and generalizes relative position trajectories using DMP, and store the learned result in .h5 file for later use.
% Targets to encode: elbow pos relative to wrist, relative wrist pos (between left and right). Three parts in total.    

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

num_datapoints = 100;%50;
num_resampled_points = 50;


%% Load demonstration
file_name = '../motion-retargeting/test_nullspace_YuMi.h5'; %test_imi_data_YuMi.h5'; %'test_imi_data_YuMi.h5'; %
group_name = 'gun_2';%'fengren_10';%'gun_2';%'kai_2';%'qie_1';%'kai_1'; %'kai_1'; %'fengren_1';

time = h5read(file_name, ['/', group_name, '/time']);

l_wrist_pos_human = resample_traj(time, h5read(file_name, ['/', group_name, '/l_wrist_pos']), num_datapoints, false);
r_wrist_pos_human = resample_traj(time, h5read(file_name, ['/', group_name, '/r_wrist_pos']), num_datapoints, false);

l_elbow_pos_human = resample_traj(time, h5read(file_name, ['/', group_name, '/l_elbow_pos']), num_datapoints, false);
r_elbow_pos_human = resample_traj(time, h5read(file_name, ['/', group_name, '/r_elbow_pos']), num_datapoints, false); % input should be DOF x length

l_shoulder_pos_human = resample_traj(time, h5read(file_name, ['/', group_name, '/l_shoulder_pos']), num_datapoints, false);
r_shoulder_pos_human = resample_traj(time, h5read(file_name, ['/', group_name, '/r_shoulder_pos']), num_datapoints, false); % original shoulder trajectories

l_wrist_ori = h5read(file_name, ['/', group_name, '/l_wrist_ori']);
r_wrist_ori = h5read(file_name, ['/', group_name, '/r_wrist_ori']);

% hand tip trajectories inferred from wrist pos and ori during pre-processing of imitation data     
l_hand_tip_pos = h5read(file_name, ['/', group_name, '/l_hand_tip_pos']);
r_hand_tip_pos = h5read(file_name, ['/', group_name, '/r_hand_tip_pos']);
human_hand_length = h5read(file_name, ['/', group_name, '/demonstrator_hand_length']); % 12 cm by default; my data (LYW); from the origin of hand marker to the tip of middle finger  

% Adjust the wrist positions according to human hand length and robotic hand length
% Process:
% human wrist position + human wrist orientation + human hand length --> human hand tip traj
% robot wrist position <-- human wrist orientation + robotic hand length <----|     
robot_hand_axis = [0, 0, 1]'; % along z-axis
robot_hand_length = 0.2; %0.278; %0.20; % 0.195 according to rviz simulation... % 27.8 cm according to documentation; (there might be a base which is not included in the model, be careful..)
l_wrist_pos_inferred = zeros(size(l_hand_tip_pos));
r_wrist_pos_inferred = zeros(size(r_hand_tip_pos));
for n = 1 : size(l_hand_tip_pos, 2)
    % get rotation matrices
    l_wrist_rot = reshape(l_wrist_ori(:, n), 3, 3)';
    r_wrist_rot = reshape(r_wrist_ori(:, n), 3, 3)'; % transpose to be RowMajor
    
    % infer robot wrist pos
    l_wrist_pos_inferred(:, n) = l_hand_tip_pos(:, n) + l_wrist_rot * robot_hand_axis * (-robot_hand_length);
    r_wrist_pos_inferred(:, n) = r_hand_tip_pos(:, n) + r_wrist_rot * robot_hand_axis * (-robot_hand_length);
    
end
l_wrist_pos_robot = resample_traj(time, l_wrist_pos_inferred, num_datapoints, false);
r_wrist_pos_robot = resample_traj(time, r_wrist_pos_inferred, num_datapoints, false);


% debug:
figure;
p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); % wrist
p2 = plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'g-');
plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'g-'); % elbow
p3 = plot3(l_shoulder_pos_human(1, :), l_shoulder_pos_human(2, :), l_shoulder_pos_human(3, :), 'y-');
plot3(r_shoulder_pos_human(1, :), r_shoulder_pos_human(2, :), r_shoulder_pos_human(3, :), 'y-'); % shoulder
p4 = plot3(l_hand_tip_pos(1, :), l_hand_tip_pos(2, :), l_hand_tip_pos(3, :), 'r-');
plot3(r_hand_tip_pos(1, :), r_hand_tip_pos(2, :), r_hand_tip_pos(3, :), 'r-'); % handtip
p5 = plot3(l_wrist_pos_robot(1, :), l_wrist_pos_robot(2, :), l_wrist_pos_robot(3, :), 'b--'); 
plot3(r_wrist_pos_robot(1, :), r_wrist_pos_robot(2, :), r_wrist_pos_robot(3, :), 'b--'); % adjusted wrist
title('Human wrist movement and modified wrist movement', 'FontSize', 16);
%view(-45, 45);
view(45, 45);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 
legend([p1(1), p2(1), p3(1), p4(1), p5(1)], ...
        'Original Wrist Traj', 'Original Elbow Traj', 'Original Shoulder Traj', ...
        'Original HandTip Traj', 'Adjusted Wrist Traj', ...
        'Location', 'NorthEastOutside');

% compute relative trajectories for DMP learning
% here the relative relationship between wrist and elbow is actually modified manually, but we could also say that relative relationsh between hand tip and elbow is preserved...     
lr_wrist_pos = l_wrist_pos_robot - r_wrist_pos_robot; % use inferred robot wrist trajectories for modeling wrist-relative relationship
l_elbow_wrist_pos = l_elbow_pos_human - l_wrist_pos_human;
r_elbow_wrist_pos = r_elbow_pos_human - r_wrist_pos_human; % use original human data for modeling elbow-relative trajectories


%% Compute and store adjusted wrist, elbow and shoulder trajectories for displaying modified results in RViz
% should be of same length, so as to compare with the original ones
ori_num_datapoints = size(time, 2);
time_expanded = resample_traj(time, time, num_datapoints, false);

% wrist
l_wrist_pos_adjusted = resample_traj(time_expanded, l_wrist_pos_robot, ori_num_datapoints, false);
r_wrist_pos_adjusted = resample_traj(time_expanded, r_wrist_pos_robot, ori_num_datapoints, false);

% elbow
l_elbow_pos_adjusted = resample_traj(time_expanded, l_wrist_pos_robot + l_elbow_wrist_pos, ori_num_datapoints, false);
r_elbow_pos_adjusted = resample_traj(time_expanded, r_wrist_pos_robot + r_elbow_wrist_pos, ori_num_datapoints, false);

% shoulder
l_shoulder_elbow_pos = l_shoulder_pos_human - l_elbow_pos_human;
r_shoulder_elbow_pos = r_shoulder_pos_human - r_elbow_pos_human; % shoulder-elbow relative traj
l_shoulder_pos_adjusted = resample_traj(time_expanded, l_wrist_pos_robot + l_elbow_wrist_pos + l_shoulder_elbow_pos, ...
                                        ori_num_datapoints, false);
r_shoulder_pos_adjusted = resample_traj(time_expanded, r_wrist_pos_robot + r_elbow_wrist_pos + r_shoulder_elbow_pos, ...
                                        ori_num_datapoints, false);

                                    
% debug: original and adjusted movements
figure;

p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'r-', 'LineWidth', 1.5); hold on; grid on;
plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-', 'LineWidth', 1.5);  % wrist
p2 = plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'g-', 'LineWidth', 1.5); 
plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'g-', 'LineWidth', 1.5); % elbow

p3 = plot3(l_shoulder_pos_human(1, :), l_shoulder_pos_human(2, :), l_shoulder_pos_human(3, :), 'y-');
plot3(r_shoulder_pos_human(1, :), r_shoulder_pos_human(2, :), r_shoulder_pos_human(3, :), 'y-'); % shoulder

p4 = plot3(l_wrist_pos_adjusted(1, :), l_wrist_pos_adjusted(2, :), l_wrist_pos_adjusted(3, :), 'b--'); 
plot3(r_wrist_pos_adjusted(1, :), r_wrist_pos_adjusted(2, :), r_wrist_pos_adjusted(3, :), 'b--'); % wrist
p5 = plot3(l_elbow_pos_adjusted(1, :), l_elbow_pos_adjusted(2, :), l_elbow_pos_adjusted(3, :), 'g--'); 
plot3(r_elbow_pos_adjusted(1, :), r_elbow_pos_adjusted(2, :), r_elbow_pos_adjusted(3, :), 'g--'); % elbow
p6 = plot3(l_shoulder_pos_adjusted(1, :), l_shoulder_pos_adjusted(2, :), l_shoulder_pos_adjusted(3, :), 'y--');
plot3(r_shoulder_pos_adjusted(1, :), r_shoulder_pos_adjusted(2, :), r_shoulder_pos_adjusted(3, :), 'y--'); % shoulder

title('Human demonstrated and Adjusted movements', 'FontSize', 16);
view(45, 45);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 
legend([p1(1), p2(1), p3(1), p4(1), p5(1), p6(1)], ...
       'Original Wrist Traj', 'Original Elbow Traj', 'Original Shoulder Traj', ...
       'Adjusted Wrist Traj', 'Adjusted Elbow Traj', 'Adjusted Shoulder Traj', ...
       'Location', 'NorthEastOutside');


%% DMP settings
nbData = num_datapoints; % resampling inside DMP_learn.m !!!
nbStates = 64; %Number of activation functions (i.e., number of states in the GMM)
nbVar = 1; %Number of variables for the radial basis functions [s] (decay term)
nbVarPos = 3; %2; %Number of motion variables [x1,x2] 
kP = 64; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
dt = 0.04; %1/num_datapoints; %Duration of time step
nbSamples = 1; % Number of samples (of the same movement)
display = false; % display the reproduction to see the performance


%% Construct traj_dataset
% relative position between left and right wrists
traj_dataset{1} = lr_wrist_pos'; % should be of size Length x DOF!!!
% 1 - Use TP-GMM to model the goal signal
% model = DMP_TPGMM(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, true); % scalability not quite well, possibily due to modeling g instead of f/x/(g-y0)
% 2 - Multiple methods, including direct inverse(best for our case), LWR, several GMR implementations, etc.      
[Mu_lrw, Sigma_lrw, Weights_lrw, sIn_lrw, Yr_lrw] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, display);
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
[Mu_lew, Sigma_lew, Weights_lew, sIn_lew, Yr_lew] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, display);
% f_lew = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);

% relative position between elbow and wrist (Right)
traj_dataset{1} = r_elbow_wrist_pos'; % should be of size Length x DOF!!!
[Mu_rew, Sigma_rew, Weights_rew, sIn_rew, Yr_rew] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, display);
% f_rew = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);

% absolute position of Right wrist
traj_dataset{1} = r_wrist_pos_robot'; % should be of size Length x DOF!!!
[Mu_rw, Sigma_rw, Weights_rw, sIn_rw, Yr_rw] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbSamples, display);
% f_rw = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha);



%% Store the learned results
%{
% robot setting
h5create(file_name, ['/', group_name, '/robot_hand_length'], size(robot_hand_length));
h5write(file_name, ['/', group_name, '/robot_hand_length'], robot_hand_length);

% adjusted movements for display in RViz using visualization_msgs markers
h5create(file_name, ['/', group_name, '/l_wrist_pos_adjusted'], size(l_wrist_pos_adjusted));
h5write(file_name, ['/', group_name, '/l_wrist_pos_adjusted'], l_wrist_pos_adjusted);
h5create(file_name, ['/', group_name, '/r_wrist_pos_adjusted'], size(r_wrist_pos_adjusted));
h5write(file_name, ['/', group_name, '/r_wrist_pos_adjusted'], r_wrist_pos_adjusted); % wrist
h5create(file_name, ['/', group_name, '/l_elbow_pos_adjusted'], size(l_elbow_pos_adjusted));
h5write(file_name, ['/', group_name, '/l_elbow_pos_adjusted'], l_elbow_pos_adjusted);
h5create(file_name, ['/', group_name, '/r_elbow_pos_adjusted'], size(r_elbow_pos_adjusted));
h5write(file_name, ['/', group_name, '/r_elbow_pos_adjusted'], r_elbow_pos_adjusted); % elbow
h5create(file_name, ['/', group_name, '/l_shoulder_pos_adjusted'], size(l_shoulder_pos_adjusted));
h5write(file_name, ['/', group_name, '/l_shoulder_pos_adjusted'], l_shoulder_pos_adjusted);
h5create(file_name, ['/', group_name, '/r_shoulder_pos_adjusted'], size(r_shoulder_pos_adjusted));
h5write(file_name, ['/', group_name, '/r_shoulder_pos_adjusted'], r_shoulder_pos_adjusted); % shoulder

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

% goals and starts
h5create(file_name, ['/', group_name, '/lrw_goal'], size(lr_wrist_pos(:, end)));
h5write(file_name, ['/', group_name, '/lrw_goal'], lr_wrist_pos(:, end));
h5create(file_name, ['/', group_name, '/lrw_start'], size(lr_wrist_pos(:, 1)));
h5write(file_name, ['/', group_name, '/lrw_start'], lr_wrist_pos(:, 1));

h5create(file_name, ['/', group_name, '/lew_goal'], size(l_elbow_wrist_pos(:, end)));
h5write(file_name, ['/', group_name, '/lew_goal'], l_elbow_wrist_pos(:, end));
h5create(file_name, ['/', group_name, '/lew_start'], size(l_elbow_wrist_pos(:, 1)));
h5write(file_name, ['/', group_name, '/lew_start'], l_elbow_wrist_pos(:, 1));

h5create(file_name, ['/', group_name, '/rew_goal'], size(r_elbow_wrist_pos(:, end)));
h5write(file_name, ['/', group_name, '/rew_goal'], r_elbow_wrist_pos(:, end));
h5create(file_name, ['/', group_name, '/rew_start'], size(r_elbow_wrist_pos(:, 1)));
h5write(file_name, ['/', group_name, '/rew_start'], r_elbow_wrist_pos(:, 1));

h5create(file_name, ['/', group_name, '/rw_goal'], size(r_wrist_pos_robot(:, end)));
h5write(file_name, ['/', group_name, '/rw_goal'], r_wrist_pos_robot(:, end));
h5create(file_name, ['/', group_name, '/rw_start'], size(r_wrist_pos_robot(:, 1)));
h5write(file_name, ['/', group_name, '/rw_start'], r_wrist_pos_robot(:, 1)); % store inferred (and resampled) robot wrist pos

%}


%% Reproduce trajectory
%{
new_goal_lrw = lr_wrist_pos(:, end);% + [0.01, -0.01, 0.05]';
new_start_lrw = lr_wrist_pos(:, 1);% + [-0.01, 0.01, 0.05]';
new_goal_lew = l_elbow_wrist_pos(:, end);% + [0.01, 0.0, -0.05]';
new_start_lew = l_elbow_wrist_pos(:, 1);% + [0.01, -0.01, -0.05]';
new_goal_rew = r_elbow_wrist_pos(:, end);% + [0.01, 0.02, 0.03]';
new_start_rew = r_elbow_wrist_pos(:, 1);% + [0.02, 0.01, 0.05]';
new_goal_rw = r_wrist_pos_robot(:, end);% + [-0.02, 0.0, 0.0]';
new_start_rw = r_wrist_pos_robot(:, 1);% + [-0.01, 0.0, 0.0]';
% y_lrw = DMP_use_f(f_lrw, nbData, alpha, kP, kV, new_goal_lrw, new_start_lrw);
% y_lew = DMP_use_f(f_lew, nbData, alpha, kP, kV, new_goal_lew, new_start_lew);
% y_rew = DMP_use_f(f_rew, nbData, alpha, kP, kV, new_goal_rew, new_start_rew);
% y_rw = DMP_use_f(f_rw, nbData, alpha, kP, kV, new_goal_rw, new_start_rw);
%}

% setting for nullspace test
%
% 1 - new initial
l_wrist_pos_new_init = [0.409259 0.181061 0.190676]';
l_elbow_pos_new_init = [0.218174 0.309546 0.377905]';
r_wrist_pos_new_init = [0.409914 -0.179371  0.190834]';
r_elbow_pos_new_init = [0.218785 -0.308805  0.377362]';
% 2 - original initial and goal
l_wrist_pos_ori_init = l_wrist_pos_robot(:, 1);
l_elbow_pos_ori_init = l_elbow_pos_human(:, 1);
r_wrist_pos_ori_init = r_wrist_pos_robot(:, 1);
r_elbow_pos_ori_init = r_elbow_pos_human(:, 1);
l_wrist_pos_ori_goal = l_wrist_pos_robot(:, end);
l_elbow_pos_ori_goal = l_elbow_pos_human(:, end);
r_wrist_pos_ori_goal = r_wrist_pos_robot(:, end);
r_elbow_pos_ori_goal = r_elbow_pos_human(:, end);
% 3 - compute offsets
l_wrist_pos_new_offset = l_wrist_pos_new_init - l_wrist_pos_ori_init;
l_elbow_pos_new_offset = l_elbow_pos_new_init - l_elbow_pos_ori_init;
r_wrist_pos_new_offset = r_wrist_pos_new_init - r_wrist_pos_ori_init;
r_elbow_pos_new_offset = r_elbow_pos_new_init - r_elbow_pos_ori_init;
% 4 - get new goals
l_wrist_pos_new_goal = l_wrist_pos_ori_goal + l_wrist_pos_new_offset;
l_elbow_pos_new_goal = l_elbow_pos_ori_goal + l_elbow_pos_new_offset;
r_wrist_pos_new_goal = r_wrist_pos_ori_goal + r_wrist_pos_new_offset;
r_elbow_pos_new_goal = r_elbow_pos_ori_goal + r_elbow_pos_new_offset;
% 5 - assign to DMP starts and goals
new_goal_lrw = l_wrist_pos_new_goal - r_wrist_pos_new_goal;
new_start_lrw = l_wrist_pos_new_init - r_wrist_pos_new_init;
new_goal_lew = l_elbow_pos_new_goal - l_wrist_pos_new_goal;
new_start_lew = l_elbow_pos_new_init - l_wrist_pos_new_init;
new_goal_rew = r_elbow_pos_new_goal - r_wrist_pos_new_goal;
new_start_rew = r_elbow_pos_new_init - r_wrist_pos_new_init;
new_goal_rw = r_wrist_pos_new_goal;
new_start_rw = r_wrist_pos_new_init;
%}

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, num_resampled_points, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, num_resampled_points, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, num_resampled_points, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, num_resampled_points, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);


%
figure; plot3(y_rw(1,:), y_rw(2,:), y_rw(3,:), 'r-.'); hold on; grid on;
plot3(r_wrist_pos_robot(1,:), r_wrist_pos_robot(2,:), r_wrist_pos_robot(3,:), 'b-.');
title('r\_wist\_pos', 'FontSize', 16);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 

figure; plot3(y_lrw(1,:), y_lrw(2,:), y_lrw(3,:), 'r-.'); hold on; grid on;
plot3(lr_wrist_pos(1,:), lr_wrist_pos(2,:), lr_wrist_pos(3,:), 'b-.');
title('lr\_wrist\_pos', 'FontSize', 16);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 

figure; plot3(y_lew(1,:), y_lew(2,:), y_lew(3,:), 'r-.'); hold on; grid on;
plot3(l_elbow_wrist_pos(1,:), l_elbow_wrist_pos(2,:), l_elbow_wrist_pos(3,:), 'b-.');
title('l\_elbow\_wrist\_pos', 'FontSize', 16);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 

figure; plot3(y_rew(1,:), y_rew(2,:), y_rew(3,:), 'r-.'); hold on; grid on;
plot3(r_elbow_wrist_pos(1,:), r_elbow_wrist_pos(2,:), r_elbow_wrist_pos(3,:), 'b-.');
title('r\_elbow\_wrist\_pos', 'FontSize', 16);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 

%}


%% display human demonstrated movement and DMP generated movement
y_r_wrist = y_rw;
y_l_wrist = y_rw + y_lrw;
y_r_elbow = y_rw + y_rew;
y_l_elbow = y_l_wrist + y_lew;

figure;
plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); 
plot3(l_shoulder_pos_human(1, :), l_shoulder_pos_human(2, :), l_shoulder_pos_human(3, :), 'y-');
plot3(r_shoulder_pos_human(1, :), r_shoulder_pos_human(2, :), r_shoulder_pos_human(3, :), 'y-');
plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'g-'); 
plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'g-');  % human demonstrated movement
 
plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--');
plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--'); % DMP generated movement

view(-45, 45);
title('Human demonstrated and DMP generated trajectories', 'FontSize', 16);
% title('Original and generalized trajectories', 'FontSize', 18);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 


% store test data for nullspace code
%
h5create(file_name, ['/', group_name, '/l_wrist_pos_nullspace'], size(y_l_wrist));
h5write(file_name, ['/', group_name, '/l_wrist_pos_nullspace'], y_l_wrist);
h5create(file_name, ['/', group_name, '/r_wrist_pos_nullspace'], size(y_r_wrist));
h5write(file_name, ['/', group_name, '/r_wrist_pos_nullspace'], y_r_wrist);
h5create(file_name, ['/', group_name, '/l_elbow_pos_nullspace'], size(y_l_elbow));
h5write(file_name, ['/', group_name, '/l_elbow_pos_nullspace'], y_l_elbow);
h5create(file_name, ['/', group_name, '/r_elbow_pos_nullspace'], size(y_r_elbow));
h5write(file_name, ['/', group_name, '/r_elbow_pos_nullspace'], y_r_elbow);
%}


%% display modified human movement(wrist inferred according to hand length) and DMP generated movement
time_resampled = resample_traj(time, time, num_resampled_points, false);
y_lew_resampled = resample_traj(time_resampled, y_lew, num_datapoints, false);
y_rew_resampled = resample_traj(time_resampled, y_rew, num_datapoints, false);


figure;
plot3(l_wrist_pos_robot(1, :), l_wrist_pos_robot(2, :), l_wrist_pos_robot(3, :), 'b-'); hold on; grid on;
plot3(r_wrist_pos_robot(1, :), r_wrist_pos_robot(2, :), r_wrist_pos_robot(3, :), 'b-'); 
plot3(l_wrist_pos_robot(1, :)+y_lew_resampled(1, :),  ...
      l_wrist_pos_robot(2, :)+y_lew_resampled(2, :),  ...
      l_wrist_pos_robot(3, :)+y_lew_resampled(3, :), 'g-');  % elbow
plot3(r_wrist_pos_robot(1, :)+y_rew_resampled(1, :), ...
      r_wrist_pos_robot(2, :)+y_rew_resampled(2, :), ...
      r_wrist_pos_robot(3, :)+y_rew_resampled(3, :), 'g-');   % human demonstrated movement

plot3(y_l_wrist(1, :), y_l_wrist(2, :), y_l_wrist(3, :), 'r--');
plot3(y_r_wrist(1, :), y_r_wrist(2, :), y_r_wrist(3, :), 'r--'); 
plot3(y_l_elbow(1, :), y_l_elbow(2, :), y_l_elbow(3, :), 'r--'); 
plot3(y_r_elbow(1, :), y_r_elbow(2, :), y_r_elbow(3, :), 'r--'); % DMP generated movement

view(-45, 45);
title('Human modified trajectories and DMP generated trajectories', 'FontSize', 16);
% title('Original and generalized trajectories', 'FontSize', 18);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 


%% Other information to store in h5
% resampled quaternion trajectories           
l_wrist_ori = h5read(file_name, ['/', group_name, '/l_wrist_ori']);
r_wrist_ori = h5read(file_name, ['/', group_name, '/r_wrist_ori']);
length = size(l_wrist_ori, 2);
% convert rotation matrices to euler angles
l_wrist_quat = zeros(4, length);
r_wrist_quat = zeros(4, length);
for i = 1:length
    % left
    rotm = reshape(l_wrist_ori(:, i), 3, 3);
    rotm = rotm';
    quat = rotm2quat(rotm);
    l_wrist_quat(:, i) = quat';
    % right
    rotm = reshape(r_wrist_ori(:, i), 3, 3);
    rotm = rotm';
    quat = rotm2quat(rotm);
    r_wrist_quat(:, i) = quat';
end
% note that output from rotm2quat() is of the form [w, x, y, z]
l_wrist_quat_resampled = resample_traj(time, l_wrist_quat, num_resampled_points, true); 
r_wrist_quat_resampled = resample_traj(time, r_wrist_quat, num_resampled_points, true); % set true for quaternion data...


% resampled glove angle trajectories
l_glove_angle_resampled = resample_traj(time, h5read(file_name, ['/', group_name, '/l_glove_angle']), num_resampled_points, false); % 14 x N
r_glove_angle_resampled = resample_traj(time, h5read(file_name, ['/', group_name, '/r_glove_angle']), num_resampled_points, false);

% l_glove_angle = h5read(file_name, ['/', group_name, '/l_glove_angle']);
% idx = [1,2,3];
% figure; 
% plot3(l_glove_angle(idx(1),:), l_glove_angle(idx(2),:), l_glove_angle(idx(3), :), 'b-'); hold on; grid on; 
% plot3(l_glove_angle_resampled(idx(1),:), l_glove_angle_resampled(idx(2),:), l_glove_angle_resampled(idx(3),:), 'r-');


% store the resampled results
h5create(file_name, ['/', group_name, '/l_wrist_quat_resampled'], size(l_wrist_quat_resampled));
h5write(file_name, ['/', group_name, '/l_wrist_quat_resampled'], l_wrist_quat_resampled);
h5create(file_name, ['/', group_name, '/r_wrist_quat_resampled'], size(r_wrist_quat_resampled));
h5write(file_name, ['/', group_name, '/r_wrist_quat_resampled'], r_wrist_quat_resampled);
h5create(file_name, ['/', group_name, '/l_glove_angle_resampled'], size(l_glove_angle_resampled));
h5write(file_name, ['/', group_name, '/l_glove_angle_resampled'], l_glove_angle_resampled);
h5create(file_name, ['/', group_name, '/r_glove_angle_resampled'], size(r_glove_angle_resampled));
h5write(file_name, ['/', group_name, '/r_glove_angle_resampled'], r_glove_angle_resampled);


%% Checking out the performance of the best tracking wrist_pos by directly calling compute_cartesian_path()
%{
% original DMP starts and goals
lrw_goal_ori = lr_wrist_pos(:, end);
lrw_start_ori = lr_wrist_pos(:, 1);
lew_goal_ori = l_elbow_wrist_pos(:, end);
lew_start_ori = l_elbow_wrist_pos(:, 1);
rew_goal_ori = r_elbow_wrist_pos(:, end);
rew_start_ori = r_elbow_wrist_pos(:, 1);
rw_goal_ori = r_wrist_pos_robot(:, end);
rw_start_ori = r_wrist_pos_robot(:, 1);

% manually set initial position
% lw_set_start = [0.409, 0.181, 0.191]'; 
le_set_start = [0.218, 0.310, 0.378]'; 
rw_set_start = [0.410, -0.179, 0.191]'; 
re_set_start = [0.218, -0.310, 0.377]';

% adjust rw start to ensure rw start and lw start symmetric to x-z plane
lw_set_start = rw_set_start + lrw_start_ori; 
dist_y = abs(lw_set_start(2) - rw_set_start(2));
lw_set_start(2) = dist_y/2;
rw_set_start(2) = -dist_y/2; 

% compute relative DMPs' starts
lrw_set_start = lw_set_start - rw_set_start;
lew_set_start = le_set_start - lw_set_start;
rew_set_start = re_set_start - rw_set_start;

% compute offsets
lrw_offset = lrw_set_start - lrw_start_ori;
lew_offset = lew_set_start - lew_start_ori;
rew_offset = rew_set_start - rew_start_ori;
rw_offset = rw_set_start - rw_start_ori;

% apply offsets to get initially set DMP starts and goals
lrw_goal_new = lrw_goal_ori + lrw_offset;
lrw_start_new = lrw_start_ori + lrw_offset;
lew_goal_new = lew_goal_ori + lew_offset;
lew_start_new = lew_start_ori + lew_offset;
rew_goal_new = rew_goal_ori + rew_offset;
rew_start_new = rew_start_ori + rew_offset;
rw_goal_new = rw_goal_ori + rw_offset;
rw_start_new = rw_start_ori + rw_offset;

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


% Use DMP to generate wrist trajs for compute_cartesian_path to track
new_goal_lrw = lrw_goal_new; new_start_lrw = lrw_start_new;
new_goal_lew = lew_goal_new; new_start_lew = lew_start_new;
new_goal_rew = rew_goal_new; new_start_rew = rew_start_new;
new_goal_rw = rw_goal_new; new_start_rw = rw_start_new;

y_lrw = DMP_use_weights(Mu_lrw, Sigma_lrw, Weights_lrw, num_resampled_points, kP, kV, alpha, dt, new_goal_lrw, new_start_lrw, false);
y_lew = DMP_use_weights(Mu_lew, Sigma_lew, Weights_lew, num_resampled_points, kP, kV, alpha, dt, new_goal_lew, new_start_lew, false);
y_rew = DMP_use_weights(Mu_rew, Sigma_rew, Weights_rew, num_resampled_points, kP, kV, alpha, dt, new_goal_rew, new_start_rew, false);
y_rw = DMP_use_weights(Mu_rw, Sigma_rw, Weights_rw, num_resampled_points, kP, kV, alpha, dt, new_goal_rw, new_start_rw, false);

y_r_wrist_set = y_rw;
y_l_wrist_set = y_rw + y_lrw;
y_r_elbow_set = y_rw + y_rew;
y_l_elbow_set = y_l_wrist_set + y_lew;


% display 
figure;
plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); 
plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-'); 
plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-');  % human demonstrated movement
plot3(y_l_wrist_set(1, :), y_l_wrist_set(2, :), y_l_wrist_set(3, :), 'r-');
plot3(y_r_wrist_set(1, :), y_r_wrist_set(2, :), y_r_wrist_set(3, :), 'r-'); 
plot3(y_l_elbow_set(1, :), y_l_elbow_set(2, :), y_l_elbow_set(3, :), 'r-'); 
plot3(y_r_elbow_set(1, :), y_r_elbow_set(2, :), y_r_elbow_set(3, :), 'r-');  % DMP initial setup for optimization
view(-45, 45);
title('Original trajectories and DMP generated initial trajectories', 'FontSize', 16);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16); 


% store for compute_cartesian_path to track
h5create(file_name, ['/', group_name, '/l_wrist_pos_to_track'], size(y_l_wrist_set));
h5write(file_name, ['/', group_name, '/l_wrist_pos_to_track'], y_l_wrist_set);

h5create(file_name, ['/', group_name, '/r_wrist_pos_to_track'], size(y_r_wrist_set));
h5write(file_name, ['/', group_name, '/r_wrist_pos_to_track'], y_r_wrist_set);

%}

