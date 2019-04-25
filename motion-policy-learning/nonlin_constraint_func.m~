function [C, Ceq] = nonlin_constraint_func(q)
%% nonlinear constraint for SQP optimization
% q is a (6*len_samples) x 1 optimization variable.

% load needed global variables
global V_lb V_ub A_ub A_lb

% Nonlinear equalities
Ceq = [];


% Nonlinear inequalities
dt = 1 / 200; % should modify it if anything changes
T = length(q)/length(V_ub); % T = 1000 here
q_reshape = reshape(q, [6, 1000]); % fill in columns first

q_reshape_vel = [diff(q_reshape, 1, 2) / dt, zeros(6, 1)]; 
q_vel = reshape(q_reshape_vel, [6000, 1]); % get q velocity

q_reshape_acc = [diff(q_reshape_vel, 1, 2) / dt, zeros(6, 1)];
q_acc = reshape(q_reshape_acc, [6000, 1]); % get q acceleration

C = [ q_vel - repmat(V_ub, T, 1);
      repmat(V_lb, T, 1) - q_vel;
      q_acc - repmat(A_ub, T, 1);
      repmat(A_lb, T, 1) - q_acc];



