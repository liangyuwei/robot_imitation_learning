function q = generate_joint_trajectory(exp_xw, exp_xe, cov_xe, q0_seq, dt)
%% This function generates the joint trajectory using Sequential Quadratic Programming.
% Given:  exp_xw, exp_xe - the expected positions for wrist and elbow, both of size 3 x len_samples;
%         q0_seq - the sequence of q0 selected based on Frechet Distance;
%         cov_xe - relaxation term;
%         dt - time duration used to compute vel and acc.
% Output: q - the generated joint angle

% load global joint limits
global P_lb P_ub V_lb V_ub A_lb A_ub
global q_last q_vel_last %dt  % used to specify vel and acc bound of the joint angle
global q_vel_last_seq


% linear equality
Aeq = [];
Beq = [];

% linear inequality
A_pos = [];
B = [ V_ub;
     -V_lb;
      A_ub;
     -A_lb]; % position bound; upper and lower joint limts are specified here, so no need of UB and LB.

for i = 1 :size(exp_xw, 2)
    A = [ A;
        (q - q_last) / dt;
        -(q - q_last) / dt]; % vel = (q - q_last)/dt; - dt is the time interval from q_last till now
    B = [ B;
        V_ub;
        -V_lb]; % how to set velocity constraint and acceleration constraint???
    
    A = [ A;
        ((q - q_last) - q_vel_last) / dt; % assume that dt is approximately the time duration between two velocity point
        -((q - q_last) - q_vel_last) / dt];
    B = [ B;
        A_ub;
        -A_lb];
    
    
    % set optimization algorithm
    OPTIONS = optimoptions('fmincon', 'Algorithm', 'SQP');
    
    % start optimization
    q = fmincon(@gen_joint_traj_loss, q0, A, B, Aeq, Beq, OPTIONS);
    
    % record results and store temporary variables
    q_vel_last = (q - q_last) / dt; % use q_last before it's overriden
    q_last = q; % use current joint q to override q_last
    q_vel_last_seq = [q_vel_last_seq; q_vel_last]; % record the current q_vel
    
end


end