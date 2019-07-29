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
%{
% marker trajectory -> human joint trajectory


% human joint trajectory -> robot joint trajectory (affine transform)
% % Way 1: Apply affine transform for a single sample of trajectory
% t_start_trans = 1; % tao
% [M, loss] = optimize_affine_transform(t_start_trans, Q{1}); % optimize sample 1; M0??? goal ???
% Q_prime = Q; % apply affine transform
% Q_prime(:, t_start_trans:end) = M' * Q(:, t_start_trans:end) + (eye(n_dof) - M') * repmat(Q(:, t_start_trans), 1, len_samples - t_start_trans + 1);

% Way 2: Use the joint trajectory data collected from RViz simulation directly
disp('==========Import the joint trajectories collected from RViz simulation==========');
file_name = 'dual_ur5_joint_trajectory_diff_start_same_goal.h5'; %'dual_ur5_joint_trajectory.h5';
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
%}


%% Modeling of robot's elbow and wrist trajectories
%{
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
    %{
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

% post processing: set all the z to 0.3
%{
for i = 1 : n_samples
    for j = 1 : 2
        tmp = wrist_traj_dataset{i, j};
        tmp(:, 3) = ones(size(tmp, 1), 1) * 0.3;
        wrist_traj_dataset{i, j} = tmp;
    end
end
%}

% display the imitation data
%{
figure;
for i = 1 : n_samples
    traj_dataset = wrist_traj_dataset{i, 1}; % wrist_traj_dataset{i};
    plot3(traj_dataset(:, 1), traj_dataset(:, 2), traj_dataset(:, 3), 'b-'); hold on;
    plot3(traj_dataset(1, 1), traj_dataset(1, 2), traj_dataset(1, 3), 'go');
    plot3(traj_dataset(end, 1), traj_dataset(end, 2), traj_dataset(end, 3), 'ro');    
end
grid on;
title('elbow trajectories');
xlabel('x'); ylabel('y'); zlabel('z');
view(140, 45);
%}

%}


% generate fake imitation trajectories of pos and pose, using waypoints
l_x_start = [0.55, 0.35, 0.4;...
             0.5, 0.35, 0.4;...
             0.45, 0.4, 0.4; ...
             0.55, 0.3, 0.3; ...
             0.45, 0.35, 0.38; ...
             0.44, 0.4, 0.25; ...
             0.48, 0.33, 0.28; ...
             ];
l_eul_start = [0, 0, -0.25*pi;... %-0.75*pi;...% modified by LYW, 2019/07/01 to show generalization of DMP
               0, 0.15*pi, -0.5*pi;...
               0.1*pi, 0.2*pi, -0.25*pi; ...
               -0.25*pi, -0.1*pi, -0.6*pi; ...
               0.25*pi, 0.1*pi, -0.4*pi; ...
               0.2*pi, -0.3*pi, -0.28*pi; ...
               -0.1*pi, -0.1*pi, -0.4*pi; ...
               ];
l_x_final = zeros(size(l_x_start)); %[0.5, 0.1, 0.35]; % set fixed final position
l_eul_final = [0, 0, -0.75*pi; ...%-0.5*pi; ... % modified by LYW, 2019/07/01 to show generalization of DMP
               0, 0.25*pi, -0.5*pi; ...
               0.15*pi, 0.1*pi, -0.25*pi; ...
              -0.1*pi, -0.1*pi, -0.55*pi; ...
               0.15*pi, -0.12*pi, -0.2*pi; ...
              -0.15*pi, 0.15*pi, -0.65*pi; ...
              -0.2*pi, 0.2*pi, -0.45*pi]; % same final pose % - xyz
l_x_mid = zeros(size(l_x_start));
l_eul_mid = l_eul_final;

% right
r_x_start = [0.45, -0.35, 0.3;...
             0.5, -0.35, 0.3; ...
             0.55, -0.35, 0.25; ...
             0.45, -0.4, 0.4; ...
             0.55, -0.36, 0.45; ...
             0.52, -0.38, 0.38; ...
             0.48, -0.34, 0.27];
r_eul_start = [0, 0, -0.25*pi;... % 0.25*pi;... % modified by LYW, 2019/07/01 to show generalization of DMP
               0, -0.2*pi, 0.25*pi; ...
               0.15*pi, -0.1*pi, 0.75*pi; ...
               -0.15*pi, 0.15*pi, 0.35*pi; ...
               0.1*pi, 0.2*pi, 0.65*pi; ...
               0.15*pi, -0.15*pi, 0.6*pi; ...
               0.1*pi, -0.12*pi, 0.48*pi];
r_x_final = zeros(size(r_x_start)); %[0.5, -0.1, 0.35]; % set fixed final position
r_eul_final = [0, 0, 0.25*pi; ... %0.5*pi; ... % modified by LYW, 2019/07/01 to show generalization of DMP
             0, -0.25*pi, 0.5*pi; ...
             -0.15*pi, -0.1*pi, 0.75*pi;...
             0.1*pi, 0.1*pi, 0.45*pi; ...
             -0.15*pi, 0.12*pi, 0.8*pi; ...
             0.15*pi, -0.15*pi, 0.35*pi; ...
             0.2*pi, -0.2*pi, 0.55*pi]; % same final pose % - xyz, same as r_eul_final
r_x_mid = zeros(size(r_x_start));
r_eul_mid = r_eul_final;

% set final point's position according to the final pose, and compute corresponding via point    
x_contact_origin = [0.5, 0, 0.35];
for i = 1 : size(l_x_start, 1)
    % compute final point's position from final point's pose
    [tmp_goal_l, tmp_goal_r] = generate_two_goals(x_contact_origin, l_eul_final(i, :));
    l_x_final(i, :) = tmp_goal_l(1:3);
    r_x_final(i, :) = tmp_goal_r(1:3);
    % set mid points
    rotm_l = eul2rotm([l_eul_final(i, 3), l_eul_final(i, 2), l_eul_final(i, 1)]); % xyz for RViz, zyx for Matlab
    tmp = l_x_final(i, :)' + rotm_l * [-0.1, 0, 0]';
    l_x_mid(i, :) = tmp'; % left
    rotm_r = eul2rotm([r_eul_final(i, 3), r_eul_final(i, 2), r_eul_final(i, 1)]); % xyz for RViz, zyx for Matlab
    tmp = r_x_final(i, :)' + rotm_r * [-0.1, 0, 0]';
    r_x_mid(i, :) = tmp'; % right
end
          
% generate
n_samples = size(l_x_start, 1);
wrist_traj_dataset = cell(n_samples, 2);
n_first_l = []; 
n_first_r = []; % record the division point
t_series = cell(n_samples, 1);
for n = 1 : n_samples
    disp(['Generating fake trajectory ', num2str(n), '/', num2str(n_samples), '...']);
    [wrist_traj_dataset{n, 1}, tmp] = generate_fake_eef_traj([l_x_start(n, :), l_eul_start(n, :)], ...
                                                      [l_x_mid(n, :), l_eul_mid(n, :)], ...
                                                      [l_x_final(n, :), l_eul_final(n, :)], ...
                                                      1000); % 500);
    n_first_l = [n_first_l, tmp]; % record the division point
    [wrist_traj_dataset{n, 2}, tmp] = generate_fake_eef_traj([r_x_start(n, :), r_eul_start(n, :)], ...
                                                      [r_x_mid(n, :), r_eul_mid(n, :)], ...
                                                      [r_x_final(n, :), r_eul_final(n, :)], ...
                                                      1000); % 500);
    n_first_r = [n_first_r, tmp]; 
    t_series{n, 1} = linspace(0, 3, 1000); %500); % 3 seconds

end
disp('Done.');


%% GTW(Generalized Time Warping) pre-processing
disp('==========Perform GTW on the trajectories==========');
% use elbow trajectories to perform time warping(or could be wrist
% trajectories, t_series!!!!!) try it !!!!
traj_dataset = wrist_traj_dataset(:, 1); %elbow_traj_dataset; % maybe it's better to perform GTW on wrist traj?? which looks more pattern-ed???
id_func = perform_gtw_on_traj(len_samples, traj_dataset); % set the total step to len_samples = 1000
% elbow_traj_dataset_aligned = cell(length(traj_dataset), 1);
wrist_traj_dataset_aligned = cell(length(traj_dataset), 2);
t_series_aligned = cell(length(traj_dataset), 1);
disp('Do time warping...');
for i = 1 : length(traj_dataset)
    
    % fix out-of-bound ceil for the last index(caused by poor precision)
    if id_func(end, i) > size(traj_dataset{i}, 1)
        id_func(end, i) = size(traj_dataset{i}, 1);
    end
    
    % initialization
%     elbow_traj_dataset_aligned{i} = zeros(size(id_func, 1), 3);
    wrist_traj_dataset_aligned{i, 1} = zeros(size(id_func, 1), 6);
    wrist_traj_dataset_aligned{i, 2} = zeros(size(id_func, 1), 6);

    t_series_aligned{i} = zeros(size(id_func, 1), 1);
    
    for j = 1 : size(id_func, 1)
        
        t = id_func(j, i); t1 = floor(t); t2 = ceil(t);
        
        if t1 < 1
            t1 = 1;
        end
        
        if t1 ~= t2 % do interpolation
            % should perform interpolation since the same t can't correspond to the same trajectory point
%             elbow_traj_dataset_aligned{i}(j, :) = (t - t1) / (t2 - t1) * (elbow_traj_dataset{i}(t2, :) - elbow_traj_dataset{i}(t1, :)) + elbow_traj_dataset{i}(t1, :);
            wrist_traj_dataset_aligned{i, 1}(j, :) = (t - t1) / (t2 - t1) * (wrist_traj_dataset{i, 1}(t2, :) - wrist_traj_dataset{i, 1}(t1, :)) + wrist_traj_dataset{i, 1}(t1, :);
            wrist_traj_dataset_aligned{i, 2}(j, :) = (t - t1) / (t2 - t1) * (wrist_traj_dataset{i, 2}(t2, :) - wrist_traj_dataset{i, 2}(t1, :)) + wrist_traj_dataset{i, 2}(t1, :);
            t_series_aligned{i}(j) =  (t - t1) / (t2 - t1) * (t_series{i}(t2) - t_series{i}(t1)) + t_series{i}(t1);            
        else % t1 == t2
%             elbow_traj_dataset_aligned{i}(j, :) = elbow_traj_dataset{i}(t1, :);
            wrist_traj_dataset_aligned{i, 1}(j, :) = wrist_traj_dataset{i, 1}(t1, :);
            wrist_traj_dataset_aligned{i, 2}(j, :) = wrist_traj_dataset{i, 2}(t1, :);
            t_series_aligned{i}(j) = t_series{i}(t1);
        end
        
    end
end
disp('Done.');


% modify left arm imitation traj 1,3,4 by hand
%{
tmp = wrist_traj_dataset_aligned{1, 1}; 
tmp(:, 1:3) = [linspace(0.53, 0.5, 800)', linspace(0.4, 0.3, 800)', linspace(0.25, 0.3, 800)';...
               linspace(0.5, 0.5, 200)', linspace(0.3, 0.16, 200)', linspace(0.3, 0.3, 200)'];
wrist_traj_dataset_aligned{1, 1} = tmp;  % // traj 1          
tmp1 = wrist_traj_dataset_aligned{3, 1};
tmp1(:, 1:3) = [linspace(0.45, 0.5, 800)', linspace(0.35, 0.3, 800)', linspace(0.22, 0.3, 800)';...
                linspace(0.5, 0.5, 200)', linspace(0.3, 0.16, 200)', linspace(0.3, 0.3, 200)'];
wrist_traj_dataset_aligned{3, 1} = tmp1; % // traj 3
tmp2 = wrist_traj_dataset_aligned{4, 1};
tmp2(:, 1:3) = [linspace(0.48, 0.5, 800)', linspace(0.4, 0.3, 800)', linspace(0.25, 0.3, 800)';...
                linspace(0.5, 0.5, 200)', linspace(0.3, 0.16, 200)', linspace(0.3, 0.3, 200)'];
wrist_traj_dataset_aligned{4, 1} = tmp2; % // traj 4
%}

% generate fake imitation trajectories(randomly)
%{
left_pose = repmat([-1.5708, 0, 0], len_samples, 1);
right_pose = repmat([1.5708, 0, 0], len_samples, 1);
left_goal = [0.5, 0.16, 0.3];
right_goal = [0.5, -0.16, 0.3];
num = 32;
wrist_traj_dataset_aligned = cell(num, 2);
for n = 1 : num
    left_start = unifrnd([0.3, 0.25, 0.2], [0.6, 0.4, 0.6], 1, 3);
    right_start = unifrnd([0.3, -0.4, 0.2], [0.6, -0.25, 0.6], 1, 3); 
    offset = [(left_goal(2)+0.1 - left_start(2)) * unifrnd(0.75, 1) + left_start(2), ...
              (right_goal(2)-0.1 - right_start(2)) * unifrnd(0.75, 1) + right_start(2)];
    % generate left and right fake trajectories
    wrist_traj_dataset_aligned{n, 1} = [gen_fake_traj(left_start, left_goal, offset(1)), left_pose];
    wrist_traj_dataset_aligned{n, 2} = [gen_fake_traj(right_start, right_goal, offset(2)), right_pose];    
end
tmp = t_series_aligned;
t_series_aligned = cell(num, 1);
for j = 1 : num
    t_series_aligned{j, 1} = tmp{randi(8), 1};
end
%}

% generate fake imitation trajectories(predefine)
%{
left_pose = repmat([-1.5708, 0, 0], len_samples, 1);
right_pose = repmat([1.5708, 0, 0], len_samples, 1);
left_goal = [0.5, 0.16, 0.3];
right_goal = [0.5, -0.16, 0.3];
l_start = [0.55, 0.4, 0.4; ...
           0.57, 0.38, 0.28; ...
           0.52, 0.32, 0.2; ...
           0.45, 0.4, 0.22; ...
           0.4, 0.35, 0.35; ...
           0.47, 0.35, 0.45];
l_via = [0.5, 0.26, 0.3; ...
         0.5, 0.26, 0.3; ...
         0.5, 0.25, 0.3; ...
         0.5, 0.3, 0.3; ...
         0.5, 0.28, 0.3; ...
         0.5, 0.3, 0.3];
r_start = [0.45, -0.4, 0.25; ...
           0.55, -0.35, 0.32; ...
           0.48, -0.4, 0.4; ...
           0.57, -0.3, 0.28; ...
           0.55, -0.38, 0.25; ...
           0.48, -0.35, 0.2];
r_via = [0.5, -0.26, 0.3; ...
         0.5, -0.26, 0.3; ...
         0.5, -0.3, 0.3; ...
         0.5, -0.25, 0.3; ...
         0.5, -0.28, 0.3; ...
         0.5, -0.26, 0.3];
num = size(l_start, 1);
wrist_traj_dataset_aligned = cell(num, 2);
for n = 1 : num
    left_start = l_start(n, :);
    right_start = r_start(n, :);
    offset = [l_via(n, 2), r_via(n, 2)];
    % generate left and right fake trajectories
    wrist_traj_dataset_aligned{n, 1} = [gen_fake_traj(left_start, left_goal, offset(1)), left_pose];
    wrist_traj_dataset_aligned{n, 2} = [gen_fake_traj(right_start, right_goal, offset(2)), right_pose];    
end
tmp = t_series_aligned;
t_series_aligned = cell(num, 1);
for j = 1 : num
    t_series_aligned{j, 1} = tmp{randi(8), 1};
end
%}


% perform GMM and GMR(using PbDlib)
%{
disp('==========Perform GMM and GMR using PbDlib==========');
disp('Compute the expected xw trajectory...');
Data = zeros(6+1, len_samples * length(wrist_traj_dataset));
Data(1, :) = repmat((1:1000)/200, 1, length(wrist_traj_dataset));
for i = 1 : length(wrist_traj_dataset)
   Data(2:7, (i-1) * len_samples + 1 : i * len_samples) = wrist_traj_dataset_aligned{i, 1}'; 
%    Data(8:end, (i-1) * len_samples + 1 : i * len_samples) = wrist_traj_dataset_aligned{i, 2}'; 
end
nbStates = 3; % number of states in GMM
nbVar = 6+1; % number of variables, e.g. [t, x, y, z]
dt = 1/200; % time step duration
nbData = len_samples; % length of each trajectory; in our case, length of the GTW-aligned trajectory
nbSamples = 8; % number of samples
tic;
expected_wrist_traj = perform_GMM_GMR_xw(Data, nbStates, nbVar, dt, nbData, nbSamples); % 3 x 1000
toc;
expected_wrist_traj = expected_wrist_traj'; % now 1000 x 3
disp('Done.');
%}

% display the 3 dimensional trajectory of the imitation data
%
figure;
display_traj_dataset = wrist_traj_dataset_aligned;
display_t = t_series_aligned;
trajId = 1;
for i = trajId:trajId%1:length(display_traj_dataset)
    subplot(3, 1, 1), plot((1:1000)/200, display_traj_dataset{i}(:, 1), 'b-'); hold on; grid on; title('x'); xlabel('t');
    subplot(3, 1, 2), plot((1:1000)/200, display_traj_dataset{i}(:, 2), 'r-'); hold on; grid on; title('y'); xlabel('t');
    subplot(3, 1, 3), plot((1:1000)/200, display_traj_dataset{i}(:, 3), 'g-'); hold on; grid on; title('z'); xlabel('t');
end
% display the euclidean trajectory; without the time information, xx_traj_dataset and xx_traj_dataset_aligned should look the same    
figure;
for i = 1 : length(display_traj_dataset)
    for j = 1 : 2
        plot3(display_traj_dataset{i, j}(:, 1), display_traj_dataset{i, j}(:, 2), display_traj_dataset{i, j}(:, 3), 'b-'); hold on; grid on;     
        plot3(display_traj_dataset{i, j}(1, 1), display_traj_dataset{i, j}(1, 2), display_traj_dataset{i, j}(1, 3), 'go'); 
        plot3(display_traj_dataset{i, j}(end, 1), display_traj_dataset{i, j}(end, 2), display_traj_dataset{i, j}(end, 3), 'ro'); 
        xlabel('x'); ylabel('y'); zlabel('z');
    end
%     pause;
end
% axis([0.4, 0.6, -0.4, 0.4, 0.1, 0.5]);
view(45, 30);
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
nbStates = 10;  % number of states/activation functions
nbVar = 1; % number of the variables for the radial basis function
nbVarPos = 6;%12;%3; % number of motion variables [x, y, z]
kP_l = 64; %64; % stiffness gain
kV_l = 16; %20; % damping gain (with ideal underdamped damping ratio)
kosi_l = kV_l / (2 * sqrt(kP_l))
kP_r = 64; %400;%50; % stiffness gain
kV_r = 16;%40;
kosi_r = kV_r / (2 * sqrt(kP_r))
alpha = 1; % decay factor
dt = 1/200; % duration of time step
nbData = len_samples; % length of each trajectory(which is why they need to be GTW-aligned for use here)
nbSamples = 1; %length(t_series_aligned); %10; % number of samples

trajId = 1;  
new_start_l = wrist_traj_dataset_aligned{trajId, 1}(1, :)';% + [0.1, 0.1, -0.1, 0, -1.5708, 0]' ; 
% new_start_l(1:3) = [0.52, 0.1, 0.42]';
% new_start_l(4:6) = [-0.7854, -0.7854, -0.7854]';
% new_goal_l = wrist_traj_dataset_aligned{trajId, 1}(end, :)' + [0.1, 0, -0.1, 0, 0, 0]';
 
new_start_r = wrist_traj_dataset_aligned{trajId, 2}(1, :)';% + [-0.1, -0.1, 0.1, 0, 1.5708, 0]'; 
% new_goal_r = wrist_traj_dataset_aligned{trajId, 2}(end, :)' + [-0.1, 0, 0.1, 0, 0, 0]';

% new way of generating goals(non-horizontal goal)
new_x_contact_origin = x_contact_origin + [0.0, 0.0, 0.05]; % new contact goal
l_euler_final = wrist_traj_dataset_aligned{trajId, 1}(end, 4:6); %[0, 0, -0.5*pi]; %[0, 0, -0.5*pi]; % xyz, for RViz
[new_goal_l, new_goal_r] = generate_two_goals(new_x_contact_origin, l_euler_final);
new_goal_l = new_goal_l';
new_goal_r = new_goal_r';
            
% perform DMP on the left arm's imitation data
%{
wrist_traj_dataset_combined = cell(length(wrist_traj_dataset_aligned), 1);
for c = 1 : length(wrist_traj_dataset_aligned)
    wrist_traj_dataset_combined{c} = [wrist_traj_dataset_aligned{c, 1}, wrist_traj_dataset_aligned{c, 2}];
end
wrist_traj_dataset_only_eul = cell(length(wrist_traj_dataset_aligned), 2);
for c = 1 : length(wrist_traj_dataset_aligned)
    wrist_traj_dataset_only_eul{c, 1} = wrist_traj_dataset_aligned{c, 1}(:, 4:6);
    wrist_traj_dataset_only_eul{c, 2} = wrist_traj_dataset_aligned{c, 2}(:, 4:6);
end
%}

%{
tic; %wrist_traj_dataset_aligned(1, 1) --------------\\
new_wrist_traj_l = perform_DMP('l_approach_1', {wrist_traj_dataset{1, 1}(1:n_first_l(1), :)}, nbStates, nbVar, nbVarPos, kP_l, kV_l, alpha, dt, nbData, nbSamples, new_goal_l, new_start_l);
new_wrist_traj_l = perform_DMP('l_insert_1', {wrist_traj_dataset{1, 1}((n_first_l(1)+1):end, :)}, nbStates, nbVar, nbVarPos, kP_l, kV_l, alpha, dt, nbData, nbSamples, new_goal_l, new_start_l);

% new_wrist_traj_l = perform_DMP_using_learned_params('l_test', ...
%                                                      nbStates, nbVar, nbVarPos, kP_l, kV_l, alpha, dt, ...
%                                                      nbData, nbSamples, new_goal_l, new_start_l);
% new_wrist_traj_l = perform_DMP_GMR01(wrist_traj_dataset_aligned(:, 1), nbStates, nbVarPos, kP_l, kV_l, alpha, dt, nbData, nbSamples, new_goal_l, new_start_l);
% new_wrist_traj_l = perform_DMP_GMR02(wrist_traj_dataset_aligned(:, 1), nbStates, nbVarPos, kP_l, kV_l, alpha, dt, nbData, nbSamples, new_goal_l, new_start_l);
% new_wrist_traj_l = perform_DMP02(wrist_traj_dataset_aligned(:, 1), nbStates, nbVar, nbVarPos, kP_l, kV_l, alpha, dt, nbData, nbSamples, new_goal_l, new_start_l);
toc;
new_wrist_traj_l = new_wrist_traj_l.Data;


tic; % wrist_traj_dataset_aligned(1, 2) ---------------------- \\
new_wrist_traj_r = perform_DMP('r_approach_1', {wrist_traj_dataset{1, 2}(1:n_first_r(1), :)}, nbStates, nbVar, nbVarPos, kP_r, kV_r, alpha, dt, nbData, nbSamples, new_goal_r, new_start_r);
new_wrist_traj_r = perform_DMP('r_insert_1', {wrist_traj_dataset{1, 2}((n_first_r(1)+1):end, :)}, nbStates, nbVar, nbVarPos, kP_r, kV_r, alpha, dt, nbData, nbSamples, new_goal_r, new_start_r);
% new_wrist_traj_r = perform_DMP_using_learned_params('r_test', ...
%                                                      nbStates, nbVar, nbVarPos, kP_r, kV_r, alpha, dt, ...
%                                                      nbData, nbSamples, new_goal_r, new_start_r);
% new_wrist_traj_r = perform_DMP_GMR01(wrist_traj_dataset_aligned(:, 2), nbStates, nbVarPos, kP_r, kV_r, alpha, dt, nbData, nbSamples, new_goal_r, new_start_r);
% new_wrist_traj_r = perform_DMP_GMR02(wrist_traj_dataset_aligned(:, 2), nbStates, nbVarPos, kP_r, kV_r, alpha, dt, nbData, nbSamples, new_goal_r, new_start_r);
% new_wrist_traj_r = perform_DMP02(wrist_traj_dataset_aligned(:, 2), nbStates, nbVar, nbVarPos, kP_r, kV_r, alpha, dt, nbData, nbSamples, new_goal_r, new_start_r);
toc;
new_wrist_traj_r = new_wrist_traj_r.Data;
%}

% generate coordinated path
rotm_l = eul2rotm([new_goal_l(6), new_goal_l(5), new_goal_l(4)]); % xyz for RViz, zyx for Matlab
tmp_l = new_goal_l(1:3) + rotm_l * [-0.1, 0, 0]'; % offset from the goal
new_mid_l = [tmp_l; new_goal_l(4:6)];
rotm_r = eul2rotm([new_goal_r(6), new_goal_r(5), new_goal_r(4)]); % xyz for RViz, zyx for Matlab
tmp_r = new_goal_r(1:3) + rotm_r * [-0.1, 0, 0]'; % offset from the goal
new_mid_r = [tmp_r; new_goal_r(4:6)];
%{
[new_wrist_traj_l_approach, new_wrist_traj_r_approach] = perform_biDMP('l_approach_1', 'r_approach_1', nbStates, nbVar, nbVarPos, ...
                                                     kP_l, kV_l, kP_r, kV_r, alpha, dt, ...
                                                     new_mid_l, new_start_l, new_mid_r, new_start_r);
%                                                      new_goal_l, new_start_l, new_goal_r, new_start_r);
new_wrist_traj_l_approach = new_wrist_traj_l_approach.Data;
new_wrist_traj_r_approach = new_wrist_traj_r_approach.Data;

[new_wrist_traj_l_insert, new_wrist_traj_r_insert] = perform_biDMP('l_insert_1', 'r_insert_1', nbStates, nbVar, nbVarPos, ...
                                                     kP_l, kV_l, kP_r, kV_r, alpha, dt, ...
                                                     new_goal_l, new_mid_l, new_goal_r, new_mid_r);
new_wrist_traj_l_insert = new_wrist_traj_l_insert.Data;
new_wrist_traj_r_insert = new_wrist_traj_r_insert.Data;

% combination of two DMPs
new_wrist_traj_l = [new_wrist_traj_l_approach, new_wrist_traj_l_insert];
new_wrist_traj_r = [new_wrist_traj_r_approach, new_wrist_traj_r_insert];
%}
% simple joining of DMP sequences using new script(a more reasonable implementation, no synchronization)
[new_wrist_traj_l, new_wrist_traj_r] = perform_biDMP_joining({'l_approach_1', 'l_insert_1'}, {'r_approach_1', 'r_insert_1'}, ...
                                                     nbStates, nbVar, nbVarPos, ...
                                                     kP_l, kV_l, kP_r, kV_r, alpha, dt, ...
                                                     [new_mid_l, new_goal_l], [new_start_l, new_mid_l], ...
                                                     [new_mid_r, new_goal_r], [new_start_r, new_mid_r]);
new_wrist_traj_l = new_wrist_traj_l.Data;
new_wrist_traj_r = new_wrist_traj_r.Data; % concatenated DMP sequence


% display the change of local frame
display_frame_change(new_wrist_traj_l, new_wrist_traj_r);
% display_frame_change(combined_new_wrist_traj_l, combined_new_wrist_traj_r);

% plot the newly generated trajectory
%
figure;
traj_plot = new_wrist_traj_l(1:3, :); %new_wrist_traj(1:3, :); %new_wrist_traj(1:3, :);
ori_traj_plot = wrist_traj_dataset_aligned{trajId, 1}'; ori_traj_plot = ori_traj_plot(1:3, :);
rotm_l = eul2rotm([new_goal_l(6), new_goal_l(5), new_goal_l(4)]); % xyz for RViz, zyx for Matlab
tmp_l = new_goal_l(1:3) + rotm_l * [-0.1, 0, 0]'; % offset from the goal
new_mid_l = [tmp_l; new_goal_l(4:6)];
exp_new_traj_plot = generate_fake_eef_traj(new_start_l', new_mid_l', new_goal_l', 500); 
exp_new_traj_plot = exp_new_traj_plot'; exp_new_traj_plot = exp_new_traj_plot(1:3, :);
% traj_plot = wrist_traj_dataset_aligned{i, 2}';% wrist_traj_dataset_aligned{i, 1}';
for p = 1 : 2
    plot3(traj_plot(1, :), traj_plot(2, :), traj_plot(3, :), 'b-'); hold on; grid on;
    plot3(traj_plot(1, 1), traj_plot(2, 1), traj_plot(3, 1), 'go');
    plot3(traj_plot(1, end), traj_plot(2, end), traj_plot(3, end), 'ro');
    plot3(ori_traj_plot(1, :), ori_traj_plot(2, :), ori_traj_plot(3, :), 'b--'); 
    plot3(ori_traj_plot(1, 1), ori_traj_plot(2, 1), ori_traj_plot(3, 1), 'go');
    plot3(ori_traj_plot(1, end), ori_traj_plot(2, end), ori_traj_plot(3, end), 'ro');
    plot3(exp_new_traj_plot(1, :), exp_new_traj_plot(2, :), exp_new_traj_plot(3, :), 'm--');%'m--'); 
    plot3(exp_new_traj_plot(1, 1), exp_new_traj_plot(2, 1), exp_new_traj_plot(3, 1), 'go');
    plot3(exp_new_traj_plot(1, end), exp_new_traj_plot(2, end), exp_new_traj_plot(3, end), 'ro');
    % draw right arm's traj
    traj_plot = new_wrist_traj_r(1:3, :); %new_wrist_traj(7:9, :);
    ori_traj_plot = wrist_traj_dataset_aligned{trajId, 2}'; ori_traj_plot = ori_traj_plot(1:3, :);
    rotm_r = eul2rotm([new_goal_r(6), new_goal_r(5), new_goal_r(4)]); % xyz for RViz, zyx for Matlab
    tmp_r = new_goal_r(1:3) + rotm_r * [-0.1, 0, 0]'; % offset from the goal
    new_mid_r = [tmp_r; new_goal_r(4:6)];
    exp_new_traj_plot = generate_fake_eef_traj(new_start_r', new_mid_r', new_goal_r', 500);
    exp_new_traj_plot = exp_new_traj_plot'; exp_new_traj_plot = exp_new_traj_plot(1:3, :);
end
% title(['kP = ', num2str(kP), ', kV = ', num2str(kV)]);
title(['kP_l = ', num2str(kP_l), ', kV_l = ', num2str(kV_l), '; kP_r = ', num2str(kP_r), ', kV_r = ', num2str(kV_r)]);
% axis([0.4, 0.6, -0.4, 0.4, 0.1, 0.5]);
view(120, 60);
% For PPT demonstration purpose:
% set(gca, 'FontSize', 16);
% view(135, 38); % 3d view
% view(0, 90); % x-y
% view(90, 0); % y-z
xlabel('x'); ylabel('y'); zlabel('z');

% plot the 3 dimensions
figure;
traj_plot_l = new_wrist_traj_l(1:3, :); %new_wrist_traj_l;
traj_plot_r = new_wrist_traj_r(1:3, :); %new_wrist_traj_r;
subplot(3, 2, 1); plot(1:1000, traj_plot_l(1, :), 'b.'); title('Left arm traj - x'); grid on;
subplot(3, 2, 2); plot(1:1000, traj_plot_r(1, :), 'b.'); title('Right arm traj - x'); grid on;
subplot(3, 2, 3); plot(1:1000, traj_plot_l(2, :), 'b.'); title('Left arm traj - y'); grid on;
subplot(3, 2, 4); plot(1:1000, traj_plot_r(2, :), 'b.'); title('Right arm traj - y'); grid on;
subplot(3, 2, 5); plot(1:1000, traj_plot_l(3, :), 'b.'); title('Left arm traj - z'); grid on;
subplot(3, 2, 6); plot(1:1000, traj_plot_r(3, :), 'b.'); title('Right arm traj - z'); grid on;
suptitle('Display of position change.');
%}
disp('Done.');


%% Save the new trajectory poitns
%{
h5create('generated_wrist_trajectories_dual.h5', '/new_wrist_traj_l', size(new_wrist_traj_l)); % create before writing
h5create('generated_wrist_trajectories_dual.h5', '/new_wrist_traj_r', size(new_wrist_traj_r)); % create before writing

h5write('generated_wrist_trajectories_dual.h5', '/new_wrist_traj_l', new_wrist_traj_l);
h5write('generated_wrist_trajectories_dual.h5', '/new_wrist_traj_r', new_wrist_traj_r);
%}

%% Generate trajectory of the right arm
%{
wrist_traj_dataset_combined = zeros(12, 10*1000);%cell(length(wrist_traj_dataset_aligned), 1);
for c = 1 : length(wrist_traj_dataset_aligned)
    wrist_traj_dataset_combined(:, (c-1)*1000+1 : c*1000) = [wrist_traj_dataset_aligned{c, 1}, wrist_traj_dataset_aligned{c, 2}]';
end
nbStates = 5;
nbData = len_samples;
nbVar = 6;
nbSamples = 10;
new_wrist_traj_r = perform_GMM_GMR_xe(wrist_traj_dataset_combined', nbStates, nbVar, dt, nbData, nbSamples, new_wrist_traj);
%}

