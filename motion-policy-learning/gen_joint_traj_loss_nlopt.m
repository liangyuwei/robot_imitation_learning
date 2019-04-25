function [val, gradient] = gen_joint_traj_loss_nlopt(q)
%% This function is used in calling NLopt library to perform SQP method.

%% Prepare variables
global exp_xw_seq exp_xe_seq 
global cov_xe_t

% Get the length of the sequence(could be 1)
T = 1000;

% Compute elbow and wrist trajectories by performing FK
q_reshape = reshape(q, [6, 1000]);
q_reshape = q_reshape'; % 1000 x 6
q_reshape_vel = [diff(q_reshape, 1, 1); zeros(1, 6)];
[xe_seq, xw_seq] = obtain_robot_traj(q_reshape); % seqs are 1000 x 3


%% Compute the cost function value
% Compute three components of the total cost function
J_wrist = 0;
J_elbow = 0;
J_smooth = 0;
W = diag([1,1,1,1,1,1]); % set equal weights on the joints
for t = 1 : T
    J_wrist = J_wrist + (exp_xw_seq(t, :) - xw_seq(t, :)) * (exp_xw_seq(t, :) - xw_seq(t, :))';
    J_elbow = J_elbow + (exp_xe_seq(t, :) - xe_seq(t, :)) * cov_xe_t(:, :, t) * (exp_xe_seq(t, :) - xe_seq(t, :))';
    J_smooth = J_smooth + q_reshape_vel(t, :) * W * q_reshape_vel(t, :)';
end

% Total cost function
val = J_wrist + J_elbow + J_smooth;


%% Compute the cost function's gradient
gradient = 

end