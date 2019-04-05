%% Setup
Q = {}; % n_samples x n_dof x Time(Length)
len_samples = 1000;
n_dof = 7;
n_samples = 1; % num of trajectory samples
t_series = 1:1000; % should be more than one sample..
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
% for a single sample of trajectory
% t_start_trans = 1; % tao
% [M, loss] = optimize_affine_transform(t_start_trans, Q{1}); % optimize sample 1; M0??? goal ???
% Q_prime = Q; % apply affine transform
% Q_prime(:, t_start_trans:end) = M' * Q(:, t_start_trans:end) + (eye(n_dof) - M') * repmat(Q(:, t_start_trans), 1, len_samples - t_start_trans + 1);

% use the joint trajectory data collected from RViz simulation directly
file_name = 'imi_joint_traj_dataset.h5';
Q_prime = read_hdf5_imi_data(file_name);

test_robot_fk(Q_prime{1, 1}); % display one trajectory at a time


%% Modeling of robot's elbow and wrist trajectories
% robot joint trajectory -> robot elbow & wrist trajectories 
[elbow_traj, wrist_traj] = FK_panda_arm(Q_prime); % here there's only one single trajectory, the whole dataset needs to be processd.

% GTW(Generalized Time Warping) pre-processing

    %Github code!!!

% Construct GMM(Gaussian Mixture Model)
K_w_t = 3; % number of submodels
GM_wrist_time = fitgmdist([wrist_traj', t_series'], K_w_t); % data - N x 4; should use the whole dataset instead of just one trajectory

% Compute expected trajectory using Gaussian Mixture Regression
%K_w_t = GM_wrist_time.NumComponents;
beta = GM_wrist_time.ComponentProportion; % 1 x K_w_t
mu_t = zeros(1, K_w_t); cov_t_t = zeros(1, K_w_t);
mu_xw = zeros(K_w_t, 3); cov_xw_t = zeros(3, K_w_t);
for k = 1:K_w_t
    % for signal t
    mu_t(k) = GM_wrist_time.mu(k, 4); % the 4th column corresponds to t element; k rows - k components(submodels)
    cov_t_t(k) = GM_wrist_time.Sigma(3, 3, k); 
    % for signal xw
    mu_xw(k, :) = GM_wrist_time.mu(k, 1:3); % the 1st-3rd columns correspond to xw
    cov_xw_t(:, k) = GM_wrist_time.Sigma(1:3, 4, k); % 3 x 1 for each submodel
end
t_normpdf = @(t) normpdf(t, mu_t, cov_t_t); % for each t, t_normpdf outputs a 1 x K_w_t
p_k_t = @(t) beta .* t_normpdf(t) / beta * t_normpdf(t)';% a vector of size 1 x K_w_t; each element corresponds to a submodel
mu_hat_k_xw = @(t) mu_xw + cov_xw_t' .* repmat((1./cov_t_t .* (t - mu_t))', 1, 3); % output mu(hat)^k_t, a matrix of size K x 3
mu_xw_at_t = p_k_t * mu_hat_k_xw; % output THE EXPECTED wrist position at time t, a vector of size 1 x 3;
% for t = ... % export THE EXPECTED trajectory(the goal position is determined during data acquisition, i.e. fixed)
%     mu_xw_at_t(....)
% end 

%% Generalization using Dynamic Motion Primitive
% Generate NEW wrist trajectory with NEW target position by Dynamic Motion Primitive   
g_f = [0, 0, 0]'; % new goal
y0 = [0, 0, 0]'; % new initial position
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

% Generate new elbow trajectory based on new wrist trajectory
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

%% Generation of joint trajectories(converted from Euclidean trajectory)
% Find appropriate initial value for optimization
Compute xe_dataset, xw_dataset from the imitation dataset..
Pick xe_goal, xw_goal from the newly generated trajectory mu_xw_prime and mu_xe_at_xw_seq
x_shoulder = [0, 0, 0]; % this should be known; Is is fixed at the initial position ? or from the last step?
q0 = find_closet_joint_configuration(x_shoulder, xe_goal, xw_goal, xe_dataset, xw_dataset);
% there is one and only one q0 for each desired (xe_goal, xw_goal)

% Joint trajectory generation using sequential quadratic programming
global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq
global q_last_seq q_last q_vel_last
q_last_seq = [];
q_last = []; % initial position???
q_vel_last = [0, 0, 0, 0, 0, 0, 0]'; % the starting velocity should be zero, i.e. static  
% what about q_last and q_last_vel at the start???
q = generate_joint_trajectory(exp_xw, exp_xe, cov_xe_at_xw_seq, q0);
clear global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq


