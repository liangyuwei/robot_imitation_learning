function q_seq = generate_joint_trajectory_using_NLopt(exp_xw, exp_xe, cov_xe, q0_seq, dt)
%% This function employs NLopt library to solve the optimization problem.
% Input: exp_xw, exp_xe - 3 x len_samples
%        cov_xe - 3 x 3 x len_samples
%        q0_seq - 
%        dt - constant duration of time step

%% Load needed global variables
global P_lb P_ub V_lb V_ub A_lb A_ub
global q_last q_vel_last %dt  % used to specify vel and acc bound of the joint angle
global q_vel_last_seq

%% Prepare variables for later use
q0 = reshape(q0_seq', [6000, 1])'; % 1 x 6000, row vector


% lower and upper bound
LB = repmat(P_lb, size(exp_xw, 2), 1)'; % 1 x 6000
UB = repmat(P_ub, size(exp_xw, 2), 1)'; % 1 x 6000



%% Use NLopt to employ SQP
stop = struct();

[q_seq, ~, ~] = nlopt_minimize_constrained(algorithm, @gen_joint_traj_loss_nlopt, f_data, fc, fc_data, LB, UB, q0, stop);






end