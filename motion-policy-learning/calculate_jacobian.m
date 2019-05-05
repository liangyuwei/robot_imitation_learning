function J = calculate_jacobian(q)
%% Compute jacobian matrix, relationship between linear/angular velocity and joint velocities.
% Compute the jacobian matrix under the configuration q!!!

% Initialize
global uLINK
J = zeros(6, 6); % 6 x num_joints

% Update joint configuration and robot's state
for i = 1:6
    uLINK(i+1).q = q(i);
end
ForwardKinematics(2);

% Calculation
target = uLINK(7).p; % target wrist position
for n = 1 : 6
   z = uLINK(n).R * uLINK(n).a;  % joint axis in world frame
   J(:, n) = [cross(z, target-uLINK(n).p); z];
end




end