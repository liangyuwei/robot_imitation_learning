function q_seq = inverse_kinematics_numerical(traj_pos, traj_pose, p_start, w_start)
%% This function employs numerical method to compute the joint angles of a trajectory.
% traj_* - 3 x len_samples
% *_start - 3 x 1

%% params
global th uLINK
th = [0,0,0,0,0,0];%[0.4310   -1.2701    1.5224   -1.8248   -1.5716    0.4310];
% update_robot_joint;
% ForwardKinematics(2);
    
%% Go to start state
err = [];
for n = 1 : 1000 % try 10 times???

    % compute jacobian matrix
    J = calculate_jacobian(th); % the calculation of J will update the robot's state!!!
    
    % compute delta end-pose 
    p_goal = [0.5, 0.35, 0.397]'; %traj_pos(:, 1); %p_start;
%     R_goal = Rodrigues(w_start, 1);
    p_now = uLINK(7).p;
%     R_now = R_goal;%uLINK(7).R;
%     dp_dw = calculate_dp_dw(p_goal, R_goal, p_now, R_now);
    dp_dw = [p_goal'-p_now', 0, 0, 0]';
%     if norm(dp_dw) < 1e-2 %1e-6
%         q_seq = 0;
%         return;
%     end
    err = [err, dp_dw(1:3)];%[err, norm(dp_dw)];
    
    % compute q displacement by 
    dq = pinv(J) * dp_dw;
        % inv(J) * dp_dw; %inverse of Jacobian
        % pinv(J) * dp_dw; % Moore-Penrose pseudoinverse of Jacobian
    
    % update joint angle
    th = th + dq';
    
end

figure;
% plot(1:length(err), err, 'b.'); grid on;
plot3(err(1, :), err(2, :), err(3, :), 'b-'); grid on; hold on;
plot3(0, 0, 0, 'ro');

% Iterate over the trajectory points
q_seq = 1;%[];

end