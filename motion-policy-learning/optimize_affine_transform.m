function [M', loss] = optimize_affine_transform(t_start_trans, Q)
%% This function opitimizes a single sample of trajectory with a given start time tao of affine transform.
%% Q - a trajectory sample, n_dof x length (length >= t_start_trans);
%% M0 - an initial value for the matrix M; (M is transposed before returned.)
%% goal - the expected end position for joints.

% set joint limts as global, which can be reused in generate_joint_trajectory.m      
global P_lb P_ub V_lb V_ub A_lb A_ub

% parameters setup
[n_dof, length] = size(Q); % M should be n_dof x n_dof
Q_d = Q(:, end); % should change it...
P_lb = [-pi, -pi, -pi, -pi, -pi, -pi, -pi]'; % joint position constraint
P_ub = [pi, pi, pi, pi, pi, pi, pi]'; 
V_lb = [-1, -1, -1, -1, -1, -1, -1]'; % joint velocity constraint
V_ub = [1, 1, 1, 1, 1, 1, 1]';
A_lb = [-1, -1, -1, -1, -1, -1, -1]'; % joint acceleration constraint
A_ub = [1, 1, 1, 1, 1, 1, 1]';
M0 = eye(n_dof); % the lower and upper bound is set on M since it is the optimization variable, so no bound here

% linear equality constraint
Aeq = (Q(length) - Q(t_start_trans))'; % expected position constraint; 1 x n
Beq = (Q_d - Q(t_start_trans))'; % 1 x n

% linear inequality constraint
dt = 1:1:length; % for a single sample % ??? use real time or simulation time(dt=1s) ??? 
% Q = [0, 1, 3, 6; 2, 3, 5, 8; 5, 6, 8, 11]; % Q = [1:2:400; 2:2:401; 3:2:402];% length = size(Q, 2);% test
Q_vel = (Q(:, 2:length) - Q(:, 1:length-1)) ./ repmat(dt(1:end-1), n_dof, 1); % should be n_dof x (length-1); actually, its better to use diff()...
Q_acc = (Q_vel(:, 2:length-1) - Q_vel(:, 1:length-2)) ./ repmat(dt(1:end-2), n_dof, 1); % should be n_dof x (length-1)
% position constraint
A = [ ( Q(:, t_start_trans:end) - repmat(Q(:, t_start_trans), 1, length - t_start_trans + 1) )';
     -( Q(:, t_start_trans:end) - repmat(Q(:, t_start_trans), 1, length - t_start_trans + 1) )'];
B = [ repmat(P_ub - Q(:, t_start_trans), 1, length - t_start_trans + 1)';
     -repmat(P_lb - Q(:, t_start_trans), 1, length - t_start_trans + 1)'];
% velocity constraint
A = [ A;
      ( Q_vel(:, t_start_trans:end) - repmat(Q_vel(:, t_start_trans), 1, (length - 1) - t_start_trans + 1) )';
     -( Q_vel(:, t_start_trans:end) - repmat(Q_vel(:, t_start_trans), 1, (length - 1) - t_start_trans + 1) )'];
B = [ B;
      repmat(V_ub - Q_vel(:, t_start_trans), 1, (length - 1) - t_start_trans + 1)';
     -repmat(V_lb - Q_vel(:, t_start_trans), 1, (length - 1) - t_start_trans + 1)'];
% acceleration constraint
A = [ A;
      ( Q_acc(:, t_start_trans:end) - repmat(Q_acc(:, t_start_trans), 1, (length - 2) - t_start_trans + 1) )';
     -( Q_acc(:, t_start_trans:end) - repmat(Q_acc(:, t_start_trans), 1, (length - 2) - t_start_trans + 1) )'];
B = [ B; 
      repmat(A_ub - Q_acc(:, t_start_trans), 1, (length - 2) - t_start_trans + 1)';
     -repmat(A_lb - Q_acc(:, t_start_trans), 1, (length - 2) - t_start_trans + 1)'];
 
% Apply optimization process
global tmp_Q
tmp_Q = Q(:, t_start_trans:end);
M = fmincon(@affine_trans_loss, M0, A, B, Aeq, Beq);
clear global tmp_Q % delete global variable from all workspace after the optimization 

end