%% Setup
Q = {}; % n_samples x n_dof x Time(Length)
len_samples = 1000;

%% Build robot models
global uLINK th
th = zeros(6, 1);
UX = [1 0 0]'; UY = [0 1 0]'; UZ = [0 0 1]';
uLINK = struct('name','body', 'mom', 0, 'child', 2, 'b', [0 0 0]', 'a', UZ, 'q', 0);
uLINK(2) = struct('name', 'shoulder_pan_joint', 'mom', 1, 'child', 3, 'b', [0 0 0.089159]', 'a', UZ, 'q', th(1));
uLINK(3) = struct('name', 'shoulder_lift_joint', 'mom', 2, 'child', 4, 'b', [0  0.13585 0]',  'a', UY, 'q', th(2));
uLINK(4) = struct('name', 'elbow_joint', 'mom', 3, 'child', 5, 'b', [0.425 -0.1197 0]', 'a', UY,'q', th(3));
uLINK(5) = struct('name', 'wrist_1_joint', 'mom', 4, 'child', 6, 'b', [0.39225 0 0]' , 'a', UY, 'q', th(4));
uLINK(6) = struct('name', 'wrist_2_joint', 'mom', 5, 'child', 7, 'b', [0 0.093 0]' ,'a', -UZ, 'q', th(5));
uLINK(7) = struct('name', 'wrist_3_joint', 'mom', 6, 'child', 8, 'b', [0 0 -0.09465]' ,'a', UY, 'q', th(6)); % child = 0 indicates that the calculation of forward kinematics should stop here
uLINK(8) = struct('name', 'ee_fixed_joint', 'mom', 7, 'child', 0, 'b', [0 0.09465 0]', 'a', UZ, 'q', 0); % eef, if necessary
% if more are needed, e.g. the panda hand's position and pose, then add more uLINK   


%% Human and machine's motion trajectory correspondence
% marker trajectory -> human joint trajectory


% human joint trajectory -> robot joint trajectory (affine transform)
% % Way 1: Apply affine transform for a single sample of trajectory
% t_start_trans = 1; % tao
% [M, loss] = optimize_affine_transform(t_start_trans, Q{1}); % optimize sample 1; M0??? goal ???
% Q_prime = Q; % apply affine transform
% Q_prime(:, t_start_trans:end) = M' * Q(:, t_start_trans:end) + (eye(n_dof) - M') * repmat(Q(:, t_start_trans), 1, len_samples - t_start_trans + 1);

% Way 2: Use the joint trajectory data collected from RViz simulation directly
disp('==========Import the joint trajectories collected from RViz simulation==========');
file_name = 'dual_ur5_joint_trajectory.h5';
[Q_dataset_l, Q_dataset_r] = read_hdf5_imi_data(file_name);
% t_series_l = Q_dataset_l(:, 2);
% t_series_r = Q_dataset_r(:, 2);

% Inspect the raw data
%{
[~, tmp_l] = obtain_robot_traj(Q_dataset_l{5, 1}, true);
[~, tmp_r] = obtain_robot_traj(Q_dataset_r{5, 1}, false);
figure; 
plot3(tmp_l(:, 1), tmp_l(:, 2), tmp_l(:, 3), 'b.'); hold on; grid on;
plot3(tmp_r(:, 1), tmp_r(:, 2), tmp_r(:, 3), 'r.');
title('Blue for left arm, Red for right arm');
xlabel('x'); ylabel('y'); zlabel('z');
%}

disp('Done');

% post-processing for dual arm imitation data: merge into one Q_dataset and t_series   
Q_dataset = merge_two_trajs(Q_dataset_l, Q_dataset_r);
t_series = Q_dataset(:, 3);

%% Modeling of robot's elbow and wrist trajectories
% robot joint trajectory -> robot elbow & wrist trajectories 
n_samples = size(Q_dataset, 1);
% elbow_traj_dataset = cell(n_samples, 1);
wrist_traj_dataset = cell(n_samples, 2);
disp('==========Obtain the elbow and wrist trajectories==========');
for n = 1:n_samples
    % display some messages
    disp(['Processing the trajectory ', num2str(n), '/', num2str(n_samples), '...']);
    
    % update the robot's state and obtain the elbow and wrist trajectories
    [~, wrist_traj_points_l] = obtain_robot_traj(Q_dataset{n, 1}, true);
    [~, wrist_traj_points_r] = obtain_robot_traj(Q_dataset{n, 2}, false);

    % Inspect the trajectories one by one
    %
    figure;
    plot3(wrist_traj_points_l(:, 1), wrist_traj_points_l(:, 2), wrist_traj_points_l(:, 3), 'b.'); hold on;
    plot3(wrist_traj_points_r(:, 1), wrist_traj_points_r(:, 2), wrist_traj_points_r(:, 3), 'r.');
    plot3(0, 0, 0, 'rx');
    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
%     axis([0.4, 0.6, 0.2, 0.4, 0.3, 0.6]);
    view(135, 45);
    %}
    
    % record the elbow and wrist trajectories
%     elbow_traj_dataset{n, 1} = elbow_traj_points;
%     wrist_traj_dataset{n, 1} = wrist_traj_points;
    wrist_traj_dataset{n, 1} = wrist_traj_points_l;
    wrist_traj_dataset{n, 2} = wrist_traj_points_r;
    
end
disp('Done.');
% display the imitation data
%{
figure;
for i = 1 : n_samples
    traj_dataset = elbow_traj_dataset{i}; % wrist_traj_dataset{i};
    plot3(traj_dataset(:, 1), traj_dataset(:, 2), traj_dataset(:, 3), 'b-'); hold on;
    plot3(traj_dataset(1, 1), traj_dataset(1, 2), traj_dataset(1, 3), 'go');
    plot3(traj_dataset(end, 1), traj_dataset(end, 2), traj_dataset(end, 3), 'ro');    
end
grid on;
title('elbow trajectories');
xlabel('x'); ylabel('y'); zlabel('z');
view(140, 45);
%}


% GTW(Generalized Time Warping) pre-processing
disp('==========Perform GTW on the trajectories==========');
% use elbow trajectories to perform time warping(or could be wrist
% trajectories, t_series!!!!!) try it !!!!
traj_dataset = wrist_traj_dataset; %elbow_traj_dataset; % maybe it's better to perform GTW on wrist traj?? which looks more pattern-ed???
id_func = perform_gtw_on_traj(len_samples, traj_dataset); % set the total step to len_samples = 1000
% elbow_traj_dataset_aligned = cell(length(traj_dataset), 1);
wrist_traj_dataset_aligned = cell(length(traj_dataset), 1);
t_series_aligned = cell(length(traj_dataset), 1);
disp('Do time warping...');
for i = 1 : length(traj_dataset)
    
    % fix out-of-bound ceil for the last index(caused by poor precision)
    if id_func(end, i) > size(traj_dataset{i}, 1)
        id_func(end, i) = size(traj_dataset{i}, 1);
    end
    
    % initialization
%     elbow_traj_dataset_aligned{i} = zeros(size(id_func, 1), 3);
    wrist_traj_dataset_aligned{i} = zeros(size(id_func, 1), 3);
    t_series_aligned{i} = zeros(size(id_func, 1), 1);
    
    for j = 1 : size(id_func, 1)
        
        t = id_func(j, i); t1 = floor(t); t2 = ceil(t);
        
        if t1 ~= t2 % do interpolation
            % should perform interpolation since the same t can't correspond to the same trajectory point
%             elbow_traj_dataset_aligned{i}(j, :) = (t - t1) / (t2 - t1) * (elbow_traj_dataset{i}(t2, :) - elbow_traj_dataset{i}(t1, :)) + elbow_traj_dataset{i}(t1, :);
            wrist_traj_dataset_aligned{i}(j, :) = (t - t1) / (t2 - t1) * (wrist_traj_dataset{i}(t2, :) - wrist_traj_dataset{i}(t1, :)) + wrist_traj_dataset{i}(t1, :);
            t_series_aligned{i}(j) =  (t - t1) / (t2 - t1) * (t_series{i}(t2) - t_series{i}(t1)) + t_series{i}(t1);            
        else % t1 == t2
%             elbow_traj_dataset_aligned{i}(j, :) = elbow_traj_dataset{i}(t1, :);
            wrist_traj_dataset_aligned{i}(j, :) = wrist_traj_dataset{i}(t1, :);
            t_series_aligned{i}(j) = t_series{i}(t1);
        end
        
    end
end
disp('Done.');

% perform GMM and GMR(using PbDlib)
disp('==========Perform GMM and GMR using PbDlib==========');
disp('Compute the expected xw trajectory...');
Data = zeros(4, len_samples * length(wrist_traj_dataset));
Data(1, :) = repmat((1:1000)/200, 1, length(wrist_traj_dataset));
for i = 1 : length(wrist_traj_dataset)
   Data(2:end, (i-1) * len_samples + 1 : i * len_samples) = wrist_traj_dataset_aligned{i}'; 
end
nbStates = 5; % number of states in GMM
nbVar = 4; % number of variables, e.g. [t, x, y, z]
dt = 1/200; % time step duration
nbData = len_samples; % length of each trajectory; in our case, length of the GTW-aligned trajectory
nbSamples = 10; % number of samples
expected_wrist_traj = perform_GMM_GMR_xw(Data, nbStates, nbVar, dt, nbData, nbSamples); % 3 x 1000
expected_wrist_traj = expected_wrist_traj'; % now 1000 x 3
disp('Done.');

% display the 3 dimensional trajectory of the imitation data
%{
figure;
display_traj_dataset = wrist_traj_dataset_aligned;
display_t = t_series_aligned;
for i = 1:length(display_traj_dataset)
    subplot(3, 1, 1), plot((1:1000)/200, display_traj_dataset{i}(:, 1), 'b-'); hold on; grid on; title('x'); xlabel('t');
    subplot(3, 1, 2), plot((1:1000)/200, display_traj_dataset{i}(:, 2), 'r-'); hold on; grid on; title('y'); xlabel('t');
    subplot(3, 1, 3), plot((1:1000)/200, display_traj_dataset{i}(:, 3), 'g-'); hold on; grid on; title('z'); xlabel('t');
end
% display the euclidean trajectory; without the time information, xx_traj_dataset and xx_traj_dataset_aligned should look the same    
figure;
for i = 1 : length(display_traj_dataset)
    plot3(display_traj_dataset{i}(:, 1), display_traj_dataset{i}(:, 2), display_traj_dataset{i}(:, 3), 'b-'); hold on; grid on;     
%     pause;
end
xlabel('x'); ylabel('y'); zlabel('z');
%}

% display the result, 3-dim plot and split-view
%{
figure;
plot3(expected_wrist_traj(:, 1), expected_wrist_traj(:, 2), expected_wrist_traj(:, 3), 'b-'); hold on; grid on;
plot3(0, 0, 0, 'rx');
xlabel('x'); ylabel('y'); zlabel('z');
figure;
subplot(3, 1, 1), plot(1:length(expected_wrist_traj), expected_wrist_traj(:, 1), 'b.'); hold on; grid on; xlabel('x');
subplot(3, 1, 2), plot(1:length(expected_wrist_traj), expected_wrist_traj(:, 2), 'r.'); hold on; grid on; xlabel('y');
subplot(3, 1, 3), plot(1:length(expected_wrist_traj), expected_wrist_traj(:, 3), 'g.'); hold on; grid on; xlabel('z');
%}

%% Generalization using Dynamic Motion Primitive
% Generate NEW wrist trajectory with NEW target position by Dynamic Motion Primitive   
% Perform DMP using PbDlib
disp('===========Perform DMP using PbDlib==========');
disp('Generate new wrist trajectory based on the given new start and new goal...');
nbStates = 5; % number of states/activation functions
nbVar = 1; % number of the variables for the radial basis function
nbVarPos = 3; % number of motion variables [x, y, z]
kP = 50;%50; % stiffness gain
kV = 40;%(2*50)^.5; %(2*kP)^.5; % damping gain (with ideal underdamped damping ratio)
alpha = 1; % decay factor
dt = 1/200; % duration of time step
nbData = len_samples; % length of each trajectory(which is why they need to be GTW-aligned for use here)
nbSamples = 10; % number of samples
% be careful with the goal and initial state, don't swap them...
new_goal = wrist_traj_dataset_aligned{1}(end, :)';% + [0, 0.1, 0]'; % why is z=0.315 changed to 0.4 after forward kinematics????? % move up 0.1 in y direction; % new goal
new_start = wrist_traj_dataset_aligned{1}(1, :)';% + [0, 0.1, 0]';  % move down 0.1 in y direction; % new initial position
new_wrist_traj = perform_DMP(wrist_traj_dataset_aligned, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbData, nbSamples, new_goal, new_start);
new_wrist_traj = new_wrist_traj.Data;
% plot the newly generated trajectory
%{
figure;
plot3(new_wrist_traj(1, :), new_wrist_traj(2, :), new_wrist_traj(3, :), 'b.'); hold on; grid on;
plot3(new_wrist_traj(1, 1), new_wrist_traj(2, 1), new_wrist_traj(3, 1), 'go');
plot3(new_wrist_traj(1, end), new_wrist_traj(2, end), new_wrist_traj(3, end), 'ro');
title(['kP = ', num2str(kP), ', kV = ', num2str(kV)]);
view(135, 45);
xlabel('x'); ylabel('y'); zlabel('z');
%}

disp('Done.');


%% Generation of joint trajectories(converted from Euclidean trajectory)
% Joint trajectory generation using sequential quadratic programming
%{
disp('==========Employ SQP to generate joint trajectory==========');
global P_lb P_ub V_lb V_ub A_lb A_ub
global q_last_seq q_last q_vel_last
P_lb = [-pi, -pi, -pi, -pi, -pi, -pi]'; P_ub = [pi, pi, pi, pi, pi, pi];
V_lb = [-pi, -pi, -pi, -pi, -pi, -pi]'; V_ub = [pi, pi, pi, pi, pi, pi];
A_lb = [-1, -1, -1, -1, -1, -1]'; A_ub = [1, 1, 1, 1, 1, 1]';
q_last_seq = [];
q_last = []; % initial position???
q_vel_last = [0, 0, 0, 0, 0, 0, 0]'; % the starting velocity should be zero, i.e. static  
dt = 1 / 200;
% what about q_last and q_last_vel at the start???
global exp_xe_seq exp_xw_seq % used for defining the loss function in the following optimization procedure.
exp_xe_seq = new_elbow_traj'; 
exp_xw_seq = new_wrist_traj';
q_seq = generate_joint_trajectory(new_wrist_traj, new_elbow_traj, cov_xe_t, q0_seq, dt);
%clear global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq

% save the result
save q_seq_result.mat q_seq 
%}

% Joint trajectory generation using sequential quadratic programming -- using NLopt   
%{
disp('==========Employ SQP using NLopt to generate joint trajectory==========');
global P_lb P_ub V_lb V_ub A_lb A_ub
global q_last_seq q_last q_vel_last
P_lb = [-pi, -pi, -pi, -pi, -pi, -pi]'; P_ub = [pi, pi, pi, pi, pi, pi];
V_lb = [-pi, -pi, -pi, -pi, -pi, -pi]'; V_ub = [pi, pi, pi, pi, pi, pi];
A_lb = [-1, -1, -1, -1, -1, -1]'; A_ub = [1, 1, 1, 1, 1, 1]';
q_last_seq = [];
q_last = []; % initial position???
q_vel_last = [0, 0, 0, 0, 0, 0, 0]'; % the starting velocity should be zero, i.e. static  
dt = 1 / 200;
% what about q_last and q_last_vel at the start???
global exp_xe_seq exp_xw_seq % used for defining the loss function in the following optimization procedure.
exp_xe_seq = new_elbow_traj'; 
exp_xw_seq = new_wrist_traj';
q_seq = generate_joint_trajectory(new_wrist_traj, new_elbow_traj, cov_xe_t, q0_seq, dt);
%clear global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq

% save the result
save q_seq_result.mat q_seq 
%}

% Joint trajectory generation using sequential quadratic programming, step by step    
%{
disp('==========Generating joint trajectory==========');
global P_lb P_ub V_lb V_ub A_lb A_ub
% global q_last_seq q_last q_vel_last
P_lb = [-pi, -pi, -pi, -pi, -pi, -pi]'; P_ub = [pi, pi, pi, pi, pi, pi]';
V_lb = [-pi, -pi, -pi, -pi, -pi, -pi]'; V_ub = [pi, pi, pi, pi, pi, pi]';
A_lb = [-1, -1, -1, -1, -1, -1]'; A_ub = [1, 1, 1, 1, 1, 1]';
% q_last_seq = [];
% q_last = []; % initial position???
% q_vel_last = [0, 0, 0, 0, 0, 0, 0]'; % the starting velocity should be zero, i.e. static 
dt = 1 / 200;
% what about q_last and q_last_vel at the start???
global exp_xe_seq exp_xw_seq % used for defining the loss function in the following optimization procedure.
% exp_xe_seq = new_elbow_traj'; 
exp_xw_seq = new_wrist_traj';
q_seq = generate_joint_trajectory_stepbystep(new_wrist_traj, [], [], [], dt);
%clear global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq

% save the result
h5create('generated_joint_trajectory.h5', '/q_seq', size(q_seq)); % create before writing
h5write('generated_joint_trajectory.h5', '/q_seq', q_seq);

% display the result
[~, xw_display] = obtain_robot_traj(q_seq');
figure;
plot3(xw_display(:, 1), xw_display(:, 2), xw_display(:, 3),'b.'); hold on;
plot3(new_wrist_traj(1, :), new_wrist_traj(2, :), new_wrist_traj(3, :), 'r.');
grid on;
title('The corresponding wrist trajectory, based on the newly generated joint trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
%}

% Joint trajectory generation by numerical solution
%
disp('==========Generating joint trajectory==========');
p_start = [0.5, 0.35, 1.3973]'; %[0, 0, 0]'; 
w_start = rot2omega(eye(3)); %[-1.2561, -1.1366, 1.1343]'; %[0, 0, -1]'; % rot2omega(R_start)
R_fixed = [0, 0, -1; 1, 0, 0; 0, -1, 0]; % - Rx(-pi/2)*Ry(-pi/2) 
        % [0, 0, 1; 0, 1, 0; -1, 0, 0]; % - R_y(pi/2) 
        % eye(3); - stay initial state
wrist_pose_traj = repmat(R_fixed, 1, 1, len_samples); %zeros(3, len_samples);
q_seq = inverse_kinematics_numerical(new_wrist_traj, wrist_pose_traj, p_start, w_start);

% Save the result
h5create('generated_joint_trajectory.h5', '/q_seq', size(q_seq)); % create before writing
h5write('generated_joint_trajectory.h5', '/q_seq', q_seq);

% display the result
[xe_display, xw_display] = obtain_robot_traj(q_seq');
figure;
plot3(xw_display(:, 1), xw_display(:, 2), xw_display(:, 3),'b.'); hold on;
plot3(new_wrist_traj(1, :), new_wrist_traj(2, :), new_wrist_traj(3, :), 'r.');
grid on;
title('The corresponding wrist trajectory, based on the newly generated joint trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
%}

