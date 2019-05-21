function [elbow_traj_points, end_traj_points] = obtain_robot_traj(q_traj, left_or_right_arm)
%% This function updates the robot's joint angles and returns its wrist and elbow trajectories.
% left_or_right_arm: true for left arm, false for right arm

% Initialize the position and pose of the base
global uLINK
if (left_or_right_arm)
    uLINK(1).R = eul2rotm([0, 0, -pi/4]);%eye(3);    
    uLINK(1).p = [-0.06, 0.235, 0.395]';
else
    uLINK(1).R = eul2rotm([0, 0, pi/4]);
    uLINK(1).p = [-0.06, -0.235, 0.395]';
end

% Set up needed parameters
global th
n = size(q_traj, 1); % number of time instances

% Iteratively compute the robot's state
end_traj_points = zeros(n, 6);
elbow_traj_points = [];%zeros(n, 3);
for m = 1:n
    
    % assign joint angles
    th = q_traj(m, :)';
    
    % update the robot's state
    update_robot_joint(left_or_right_arm);
    ForwardKinematics(2);
    
    % record trajectory points
    end_traj_points(m, :) = [uLINK(8).p', rotm2eul(uLINK(8).R)]; % add in the information about pose
%     elbow_traj_points(m, :) = uLINK(4).p';

    % plot the robot
    %{
    figure;
    for i = 1 : 6
        plot3([uLINK(i).p(1), uLINK(i+1).p(1)], [uLINK(i).p(2), uLINK(i+1).p(2)], [uLINK(i).p(3), uLINK(i+1).p(3)], 'b-'); hold on; grid on;
    end
    xlabel('x'); ylabel('y'); zlabel('z');
    %}
    
end


end





