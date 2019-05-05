function q_seq = generate_joint_trajectory_stepbystep(exp_xw, exp_xe, cov_xe, q0_seq, dt)
%% This function generates the joint trajectory using Sequential Quadratic Programming.
% Given:  exp_xw, exp_xe - the expected positions for wrist and elbow, both of size 3 x len_samples;
%         q0_seq - the sequence of q0 selected based on Frechet Distance;
%         cov_xe - relaxation term;
%         dt - time duration used to compute vel and acc.
% Output: q_seq - the sequence of the generated joint angle

% Used for storing the joint trajectory
q_seq = zeros(6, 1000);

% load global joint limits
global P_lb P_ub V_lb V_ub A_lb A_ub

% record the information from the last 
q_last = [0, 0, 0, 0, 0, 0]';
q_vel_last = [0, 0, 0, 0, 0, 0]';

% linear equality
Aeq = [];
Beq = [];

% linear inequality
A = [ eye(6); 
     -eye(6); 
      eye(6); 
     -eye(6)]; 
 
% iterate to compute joint angles    
global cur_exp_xw
T = size(q0_seq, 2);
for t = 1 : T

    % display messages
    disp(['Computing joint angles for trajectory point ', num2str(t), '/', num2str(T), '...']);
    
    % nonlinear inequality constraint
    B = [  V_ub * dt + q_last;
         -(V_lb * dt + q_last);
           A_ub * dt ^ 2 + q_vel_last * dt + q_last;
         -(A_lb * dt ^ 2 + q_vel_last * dt + q_last)];

    % start optimization
    cur_exp_xw = exp_xw(:, t);
    [q, ~] = fmincon(@gen_joint_traj_loss_stepbystep, [0, 0, 0, 0, 0, 0]', A, B, Aeq, Beq, P_lb, P_ub);  % q0_seq(:, t); q_last
    % use q0_seq as the initial? or maybe use q_last?
    
    % record results and store temporary variables
    q_vel_last = (q - q_last) / dt; % use q_last before it's overriden
    q_last = q; % use current joint q to override q_last
    
    % record the current joint angles
    q_seq(:, t) = q;

end


end