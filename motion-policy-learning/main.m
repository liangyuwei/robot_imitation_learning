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
uLINK(7) = struct('name', 'wrist_3_joint', 'mom', 6, 'child', 0, 'b', [0 0 -0.09465]' ,'a', UY, 'q', th(6)); % child = 0 indicates that the calculation of forward kinematics should stop here
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
file_name = 'imi_joint_traj_dataset.h5';
Q_dataset = read_hdf5_imi_data(file_name);
t_series = Q_dataset(:, 2);
disp('Done');

%% Modeling of robot's elbow and wrist trajectories
% robot joint trajectory -> robot elbow & wrist trajectories 
n_samples = size(Q_dataset, 1);
elbow_traj_dataset = cell(n_samples, 1);
wrist_traj_dataset = cell(n_samples, 1);
disp('==========Obtain the elbow and wrist trajectories==========');
for n = 1:n_samples
    % display some messages
    disp(['Processing the trajectory ', num2str(n), '/', num2str(n_samples), '...']);
    
    % update the robot's state and obtain the elbow and wrist trajectories
    [elbow_traj_points, wrist_traj_points] = obtain_robot_traj(Q_dataset{n, 1});

    % Inspect the trajectories one by one
    %{
    figure;
    plot3(wrist_traj_points(:, 1), wrist_traj_points(:, 2), wrist_traj_points(:, 3), 'b.'); hold on;
    plot3(elbow_traj_points(:, 1), elbow_traj_points(:, 2), elbow_traj_points(:, 3), 'b.');
    plot3(0, 0, 0, 'rx');
    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    axis([0, 0.8, -0.4, 0.4, 0, 0.7]);
    view(135, 45);
    pause;
    %}
    
    % record the elbow and wrist trajectories
    elbow_traj_dataset{n, 1} = elbow_traj_points;
    wrist_traj_dataset{n, 1} = wrist_traj_points;
    
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
elbow_traj_dataset_aligned = cell(length(traj_dataset), 1);
wrist_traj_dataset_aligned = cell(length(traj_dataset), 1);
t_series_aligned = cell(length(traj_dataset), 1);
disp('Do time warping...');
for i = 1 : length(traj_dataset)
    
    % fix out-of-bound ceil for the last index(caused by poor precision)
    if id_func(end, i) > size(traj_dataset{i}, 1)
        id_func(end, i) = size(traj_dataset{i}, 1);
    end
    
    % initialization
    elbow_traj_dataset_aligned{i} = zeros(size(id_func, 1), 3);
    wrist_traj_dataset_aligned{i} = zeros(size(id_func, 1), 3);
    t_series_aligned{i} = zeros(size(id_func, 1), 1);
    
    for j = 1 : size(id_func, 1)
        
        t = id_func(j, i); t1 = floor(t); t2 = ceil(t);
        
        if t1 ~= t2 % do interpolation
            % should perform interpolation since the same t can't correspond to the same trajectory point
            elbow_traj_dataset_aligned{i}(j, :) = (t - t1) / (t2 - t1) * (elbow_traj_dataset{i}(t2, :) - elbow_traj_dataset{i}(t1, :)) + elbow_traj_dataset{i}(t1, :);
            wrist_traj_dataset_aligned{i}(j, :) = (t - t1) / (t2 - t1) * (wrist_traj_dataset{i}(t2, :) - wrist_traj_dataset{i}(t1, :)) + wrist_traj_dataset{i}(t1, :);
            t_series_aligned{i}(j) =  (t - t1) / (t2 - t1) * (t_series{i}(t2) - t_series{i}(t1)) + t_series{i}(t1);            
        else % t1 == t2
            elbow_traj_dataset_aligned{i}(j, :) = elbow_traj_dataset{i}(t1, :);
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

% Construct GMM(Gaussian Mixture Model)
% disp('==========Construct GMM models==========');
%{
K_w_t = 5; %10;%3; % 4; % number of submodels
wrist_traj_total = zeros(len_samples * length(wrist_traj_dataset), 3); %zeros(length(wrist_traj_dataset_aligned), 3*len_samples); %
t_series_total = zeros(len_samples * length(wrist_traj_dataset), 1); %zeros(length(wrist_traj_dataset_aligned), len_samples); %
for i = 1 : length(wrist_traj_dataset_aligned)
    wrist_traj_total((i-1) * len_samples + 1 : i * len_samples, :) = wrist_traj_dataset_aligned{i};
    % already tried another way of modeling GMM, but it doesn't work because there are more rows than columns   
%     tmp = wrist_traj_dataset_aligned{i}';
%     wrist_traj_total(i, :) = tmp(:)';
%     t_series_total(i, :) = 1:len_samples;
    t_series_total((i-1) * len_samples + 1 : i * len_samples) = t_series_aligned{i}; %(1:len_samples)/200; % % 1:len_samples;% % should use new time sequence???
end
GM_wrist_time = fitgmdist([wrist_traj_total, t_series_total], K_w_t); % data - N x 4; should use the whole dataset instead of just one trajectory
disp('Done.');
% save GM_wrist_time_18c_1 GM_wrist_time
%}

% Compute expected trajectory using Gaussian Mixture Regression
% disp('==========Compute expected wrist trajectory using Gaussian Mixture Regression==========');
%{
%K_w_t = GM_wrist_time.NumComponents;
beta = GM_wrist_time.ComponentProportion; % 1 x K_w_t
mu_t = zeros(1, K_w_t); cov_t_t = zeros(1, K_w_t);
mu_xw = zeros(K_w_t, 3); cov_xw_t = zeros(3, K_w_t);
% compute statistical properties
for k = 1:K_w_t
    % for signal t
    mu_t(k) = GM_wrist_time.mu(k, 4); % the 4th column corresponds to t element; k rows - k components(submodels)
    cov_t_t(k) = GM_wrist_time.Sigma(3, 3, k); 
    % for signal xw
    mu_xw(k, :) = GM_wrist_time.mu(k, 1:3); % the 1st-3rd columns correspond to xw
    cov_xw_t(:, k) = GM_wrist_time.Sigma(1:3, 4, k); % 3 x 1 for each submodel
end
% compute the expected wrist trajectory
t_normpdf_func = @(t) normpdf(t, mu_t, sqrt(cov_t_t)); % for each t, t_normpdf outputs a 1 x K_w_t; should use standard deviation!!!!! Not variance!!!!!!
% cov_t_t too small that the probability density is too focused... how to fix it ???    
time_interval = (1:len_samples)/200; % seconds; should be within t?? or use warped time sequence?
expected_wrist_traj = zeros(length(time_interval), 3);
% test the pdf function's output
test = [];
for i = 1:length(time_interval)
    test = [test; t_normpdf_func(time_interval(i))];
end
figure;
for i = 1 : size(test, 2)
    plot((1:len_samples)/200, test(:, i), 'b.'); hold on; grid on;
end
%}
% plot(1:length(time_interval), test(:, 1), 'b.'); hold on; grid on;
% plot(1:length(time_interval), test(:, 2), 'r.');
% plot(1:length(time_interval), test(:, 3), 'g.');
% plot(1:length(time_interval), test(:, 4), 'c.');
% plot(1:length(time_interval), test(:, 5), 'y.');
% axis([1, length(time_interval)+1, 0, 2]);

%{
for i = 1:length(time_interval) % export THE EXPECTED trajectory(the goal position is determined during data acquisition, i.e. fixed)
    t = time_interval(i);
    t_normpdf = t_normpdf_func(t); % 1 x K_w_t
    
    p_k_t = beta .* t_normpdf / (beta * t_normpdf');% a vector of size 1 x K_w_t; each element corresponds to a submodel; cannot start from t=0???
    
%     mu_hat_k_xw = zeros(K_w_t, 3);
%     for k = 1 : K_w_t
    mu_hat_k_xw = mu_xw + cov_xw_t' .* repmat((1./cov_t_t .* (t - mu_t))', 1, 3); % output mu(hat)^k_t, a matrix of size K x 3 
%         mu_hat_k_xw(k, :) = mu_xw(k, :) + cov_xw_t(:, k)' / cov_t_t(k) * (t - mu_t(k));
%     end
    mu_xw_at_t = p_k_t * mu_hat_k_xw; % output THE EXPECTED wrist position at time t, a vector of size 1 x 3;
    
    % record the newly generated trajectory point
    expected_wrist_traj(i, :) = mu_xw_at_t;
    
end 
%}

% display the result�� 3-dim plot and split-view
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

% code programmed by me...
%{
g_f = [0.5, 0.35, 0.315]' + [0, 0.1, 0]'; % move up 0.1 in y direction; % new goal
y0 = [0.5, -3.5, 0.315]' + [0, 0.1, 0]';  % move down 0.1 in y direction; % new initial position
% global dcps; % declare the global variable dcps for use
ID = [1, 2, 3]; n_rfs = 10; name = ['x', 'y', 'z'];
dcp('Clear', ID); % clear the data
dcp('Init', ID(1), n_rfs, name(1), 1); % initialize a DCP model(2nd order), for x element
dcp('Init', ID(2), n_rfs, name(2), 1); % for y element
dcp('Init', ID(3), n_rfs, name(3), 1); % for z element
tau = 15; % roughly movement time
dt = 0.01; % sample time step in given trajectory
dcp('Batch_Fit', ID(1), tau, dt, mu_xw_at_t(:, 1)); %T, Td, Tdd); % fit data % 3-dim ??? % the end state is automatically chosen as the goal
dcp('Batch_Fit', ID(2), tau, dt, mu_xw_at_t(:, 2));
dcp('Batch_Fit', ID(3), tau, dt, mu_xw_at_t(:, 3));
dcp('Reset_State', ID); 
dcp('Set_Goal', ID, gf, 1); % set a new goal instead of the one in the training data; test the fit
wrist_traj_new = zeros(2 * tau / dt + 1, 3);
for i = 0 : 2 * tau / dt % two times, for displaying the whole process
    % generate new point
    [x, xd, xdd] = dcp('Run', ID(1), tau, dt);
    [y, yd, ydd] = dcp('Run', ID(2), tau, dt);
    [z, zd, zdd] = dcp('Run', ID(3), tau, dt);
    wrist_traj_new(i, :) = [x, y, z];
  
end
%}
% wrist_traj_new = [];
disp('Done.');

% Generate new elbow trajectory based on new wrist trajectory
disp('Generate new elbow trajectory based on the newly wrist trajectory')
Data = zeros(nbVar, len_samples * length(wrist_traj_dataset_aligned));
for i = 1 :length(wrist_traj_dataset_aligned)
    Data(1:3, (i-1)*len_samples+1 : i*len_samples) = wrist_traj_dataset_aligned{i}';
    Data(4:6, (i-1)*len_samples+1 : i*len_samples) = elbow_traj_dataset_aligned{i}';
end
nbStates = 5; % number of states/components in GMM
nbVar = 6; % number of variables
dt = 1/200; % duration of time step
nbData = len_samples; % length of each trajectory
nbSamples = length(wrist_traj_dataset_aligned);
xw_in = new_wrist_traj; % the expected wrist traj, used to generate corresponding elbow trajectory
[xe_out, sigma_out] = perform_GMM_GMR_xe(Data, nbStates, nbVar, dt, nbData, nbSamples, xw_in);
new_elbow_traj = xe_out;
global cov_xe_t % used for defining loss function in optimization
cov_xe_t = sigma_out; % 3 x 3 x len_samples

% plot the newly generated trajectory
%{
figure;
plot3(new_elbow_traj(1, :), new_elbow_traj(2, :), new_elbow_traj(3, :), 'b.'); hold on; grid on;
plot3(new_elbow_traj(1, 1), new_elbow_traj(2, 1), new_elbow_traj(3, 1), 'go');
plot3(new_elbow_traj(1, end), new_elbow_traj(2, end), new_elbow_traj(3, end), 'ro');
view(135, 45);
title('new elbow trajectory');
xlabel('x'); ylabel('y'); zlabel('z');
%}

%{
%K_e_w = GM_elbow_wrist.NumComponents; % number of submodels
K_e_w = 3;
GM_elbow_wrist = fitgmdist([elbow_traj', wrist_traj_new], K_e_w); % data - N x 6; use the NEW TRAJ generated by DMP to model GMM!!!
pi_k = GM_elbow_wrist.ComponentProportion; % 1 x K_e_w
mu_xe = GM_elbow_wrist.mu(:, 1:3); % K_e_w x 3
mu_xw = GM_elbow_wrist.mu(:, 4:6); % K_e_w x 3
cov_xe_xw = GM_elbow_wrist.Sigma(1:3, 4:6, :); % 3 x 3 x K_e_w 
cov_xw_xw = GM_elbow_wrist.Sigma(4:6, 4:6, :); % 3 x 3 x K_e_w
cov_xe_xe = GM_elbow_wrist.Sigma(1:3, 1:3, :); % 3 x 3 x K_e_w
mu_hat_xe = zeros(K_e_w, 3); 
cov_hat_xe = zeros(3, 3, K_e_w);
xw_given = [0, 0, 0]; % the whole trajectory of xw should all be processed in this way; of size 1 x 3
pi_N = zeros(1, K_e_w);
for k = 1:K_e_w % the mu_hat_xe has to be calculated in this way in that the inverse of a set of matrix cannot be done in a batch way
    mu_hat_xe(k, :) = mu_xe(k, :) +  (cov_xe_xw(:, :, k)' * inv(cov_xw_xw(:, :, k)) * (xw_given - mu_xw)' )'; 
    % the dimension operation here is a little bit weird, should check it
    % carefully later.
    pi_N(k) = pi_k(k) * mvnpdf(xw_given, mu_xw(k, :), cov_xw_xw(:, :, k));
    cov_hat_xe(:, :, k) = cov_xe_xe(:, :, k) - cov_xe_xw(:, :, k)' * inv(cov_xw_xw(:, :, k)) * cov_xe_xw(:, :, k);
end
pi_k_xw = pi_N / sum(pi_N); % 1 x K_e_w
% two import outputs
mu_xe_at_xw = pi_k_xw * mu_hat_xe; % output a elbow position point based on the given wrist position point
cov_xe_at_xw = zeros(3, 3);
for k = 1:K_e_w
    cov_xe_at_xw = cov_xe_at_xw + pi_k_xw(k) ^2 * cov_hat_xe(:, :, k);
end
%}

%% Generation of joint trajectories(converted from Euclidean trajectory)
disp('==========Find closest joint configurations from the imitation dataset==========');
% Find appropriate initial value for optimization
% Compute xe_dataset, xw_dataset from the imitation dataset..
% Pick xe_goal, xw_goal from the newly generated trajectory mu_xw_prime and mu_xe_at_xw_seq
x_shoulder = uLINK(2).p; % try shoulder pan joint %[0, 0, 0]; % this should be known; Is is fixed at the initial position ? or from the last step?
% use aligned dataset or what??? 
best_q0_index = zeros(len_samples, 2); % [id for sample, id for time]
for t = 1 : len_samples
    disp(['Processing new trajectory point at t = ', num2str(t), '/', num2str(len_samples)]);
    xe_target = new_elbow_traj(:, t);
    xw_target = new_wrist_traj(:, t);
    best_q0_index(t, :) = find_closet_joint_configuration(x_shoulder, xe_target, xw_target, ...
                                                          elbow_traj_dataset, wrist_traj_dataset);   
end
% get the corresponding joint angles based on the indices
q0_seq = zeros(len_samples, 6);
for i = 1 :len_samples
     q_tmp = Q_dataset{best_q0_index(i, 1)};
     q0_seq(i, :) = q_tmp(best_q0_index(i, 2), :);
end
disp('Done.');

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
exp_xe_seq = new_elbow_traj'; 
exp_xw_seq = new_wrist_traj';
q_seq = generate_joint_trajectory_stepbystep(new_wrist_traj, new_elbow_traj, cov_xe_t, q0_seq', dt);
%clear global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq

% save the result
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

% Joint trajectory generation by numerical solution
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
exp_xe_seq = new_elbow_traj'; 
exp_xw_seq = new_wrist_traj';
q_seq = inverse_kinematics_numerical(new_wrist_traj, p_start, w_start);
q_seq = generate_joint_trajectory_stepbystep(new_wrist_traj, new_elbow_traj, cov_xe_t, q0_seq', dt);
%clear global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq

% save the result
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


