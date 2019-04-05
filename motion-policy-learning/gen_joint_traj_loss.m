function loss = gen_joint_traj_loss(q)
%% This function computes the cost function for generating the joint trajectory

global exp_xw_seq exp_xe_seq cov_xe_at_xw_seq V_ub

% Get the length of the sequence(could be 1)
T = size(q, 2);

% Perform forward kinematics on the joint trajectory
[elbow_traj, wrist_traj] = FK_panda_arm(q); % q, a sequence of joint angle values

% Compute three components of the total cost function
J_wrist = sum((wrist_traj - exp_xw_seq)' * (wrist_traj - exp_xw_seq)); % remember to check the dimension!!!
J_elbow = sum((elbow_traj - exp_xe_seq)' * inv(cov_xe_at_xw_seq) * (elbow_traj - exp_xe_seq)); % check the dimension!!! 
k = 1; W = eye(7) * k ./ V_ub; % diagonal matrix, where each element is in inverse proportion to the corresponding joint's maximum angular velocity
global q_vel_last_seq
J_smooth = sum()

% Total cost function
loss = J_wrist + J_elbow + J_smooth;


end