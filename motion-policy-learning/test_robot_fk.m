function test_robot_fk(q_traj)
%% This function updates the robot's joint angles and displays its motion.

% Initialize the position and pose of the base
global uLINK
uLINK(1).R = eye(3);
uLINK(1).p = [0 0 0]';

% Set up needed parameters
global th
n = size(q_traj, 1); % number of time instances

% Iteratively compute the robot's state
traj_points = zeros(n, 3);
for m = 1:n
    
    % assign joint angles
    th = q_traj(m, :);
    
    % update the robot's state
    update_robot_joint;
    ForwardKinematics(2);
    
    % record trajectory points
    traj_points(m, :) = uLINK(7).p';
end

% Display the result
figure;
plot3(traj_points(:, 1), traj_points(:, 2), traj_points(:, 3), 'b.'); hold on;
plot3(0, 0, 0, 'rx');
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
axis([0, 0.8, -0.4, 0.4, 0, 0.7]);
view(135, 45);
end





