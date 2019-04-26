function q_seq = inverse_kinematics_numerical(traj_pos, traj_pose, p_start, w_start)
%% This function employs numerical method to compute the joint angles of a trajectory.
% traj - 3 x len_samples
%

% Go to start state
dp = p_start;
dw = w_start;
for n = 1 : 10 % try 10 times???
    % compute jacobian matrix
    J = calculate_jacobian();
    
    % compute delta end-pose 
    dp_dw = calculate_dp_dw([p_start, w_start]-);
    
end


% Iterate over the trajectory points


end