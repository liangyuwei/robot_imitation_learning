function q_seq = inverse_kinematics_numerical(traj_pos, traj_pose, p_start, w_start)
%% This function employs numerical method to compute the joint angles of a trajectory.
% traj_pos - 3 x len_samples
% traj_pose - 3 x 3 x len_samples (rotation matrices)
% *_start - 3 x 1

%% setup
global th uLINK
th = [0,0,0,0,0,0];%[0.4310   -1.2701    1.5224   -1.8248   -1.5716    0.4310];
q_seq = zeros(6, size(traj_pos, 2));

%% Go to start state
%{
err = [];
for n = 1 : 1000 % try 10 times???

    % compute jacobian matrix
    J = calculate_jacobian(th); % the calculation of J ***WILL UPDATE*** the robot's state!!!
    
    % compute delta end-pose 
    p_goal = [0.5, -0.35, 0.397]'; %traj_pos(:, 1); %p_start;
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

% figure;
% % plot(1:length(err), err, 'b.'); grid on;
% plot3(err(1, :), err(2, :), err(3, :), 'b-'); grid on; hold on;
% plot3(0, 0, 0, 'ro');
%}

%% Iterate over the trajectory points
for t = 1 : size(traj_pos, 2)
    
    disp(['Processing trajectory point ', num2str(t), '/', num2str(size(traj_pos, 2)), '...']);
    p_goal = traj_pos(:, t);
    R_goal = traj_pose(:, :, t); %Rodrigues(traj_pose(:, t), 1); % what is traj_pose(:, t) exactly??? % eye(3); % 

    err = [];
    
    for n = 1 : 100 % try 1000 times to get to the target position and pose
                
        % jacobian under configuration th (will update robot's state based on th)
        J = calculate_jacobian(th);
        
        % position and pose under configuration th
        p_now = uLINK(7).p;
        R_now = uLINK(7).R; % position and pose from the last point    
    
        % generalized velocity error
        dp_dw = calculate_dp_dw(p_goal, R_goal, p_now, R_now);
        %dp_dw = [p_goal'-p_now', 0, 0, 0]';
        if norm(dp_dw) < 1e-6
            disp('Solution found.'); % plot message if solution found early
            break;
        end        
        err = [err, norm(dp_dw)]; 
%         err = [err, dp_dw(1:3)];

        % compute q error
        dq = pinv(J) * dp_dw;
        
        % update joint angle
        th = th + dq';
        
    end    
    
    % display the error converging process
%    figure;   
%    plot3(err(1, :), err(2, :), err(3, :), 'b-'); grid on; hold on;
%    plot3(0, 0, 0, 'ro');
%     figure;
%     plot(1:length(err), err, 'b.'); grid on;

    % record the result
    th_tmp = mod(th + pi, 2 * pi) - pi; % switch from [-pi, pi] to [0, 2*pi] first, and then compute the modules of it, and in the end switch back to [-pi, pi] 
    q_seq(:, t) = th_tmp';
    
    % update the current joint configuration
    th = th_tmp;

end

% 
% th_cut
% -0.7907   -1.2228    1.5642   -2.3454   -0.4133    1.9542
% 
% mod(th+pi, 2*pi)-pi  % switch from [-pi, pi] to [0, 2*pi] first, and then compute the modules of it, and in the end switch back to [-pi, pi]
% -0.7907   -1.2228    1.5642   -2.3454   -0.4133    1.9542




end