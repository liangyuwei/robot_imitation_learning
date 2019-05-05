function [elbow_traj_points, end_traj_points] = obtain_robot_traj(q_traj)
%% This function updates the robot's joint angles and returns its wrist and elbow trajectories.

% Initialize the position and pose of the base
global uLINK
uLINK(1).R = eye(3);
uLINK(1).p = [0 0 0]';

% Set up needed parameters
global th
n = size(q_traj, 1); % number of time instances

% Iteratively compute the robot's state
end_traj_points = zeros(n, 3);
elbow_traj_points = zeros(n, 3);
for m = 1:n
    
    % assign joint angles
    th = q_traj(m, :);
    
    % update the robot's state
    update_robot_joint;
    ForwardKinematics(2);
    
    % record trajectory points
    end_traj_points(m, :) = uLINK(7).p'; %uLINK(7).p';
    elbow_traj_points(m, :) = uLINK(4).p';
    
end


end





