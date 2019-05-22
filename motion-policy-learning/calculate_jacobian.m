function J = calculate_jacobian(q, left_or_right_arm)
%% Compute jacobian matrix, relationship between linear/angular velocity and joint velocities.
% Compute the jacobian matrix under the configuration q!!!

% Initialize
global uLINK
J = zeros(6, 6); % 6 x num_joints

% Update joint configuration and robot's state
if(left_or_right_arm)
    uLINK(1).R = eul2rotm([0, 0, -pi/4]);%eye(3);    
    uLINK(1).p = [-0.06, 0.235, 0.395]';
    offset = [0, -pi/4, pi/2, 0, pi/2, 0];
else
    uLINK(1).R = eul2rotm([0, 0, pi/4]);
    uLINK(1).p = [-0.06, -0.235, 0.395]';
    offset =  [pi, -0.75*pi, -pi/2, -pi, -pi/2, 0];
end
for i = 1:6
    uLINK(i+1).q = q(i) + offset(i);
end
ForwardKinematics(2);

% Calculation
target = uLINK(7).p; % target wrist position
for n = 2 : 7 % don't make mistake!!
   z = uLINK(n).R * uLINK(n).a;  % joint axis in world frame
   J(:, n-1) = [cross(z, target-uLINK(n).p); z];
end


end