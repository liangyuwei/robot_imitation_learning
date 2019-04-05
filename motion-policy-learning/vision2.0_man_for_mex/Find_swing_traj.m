function [th_inv,position_inv,ref_inv, position_left_ankle] = Find_swing_traj(P_end, th_init) 
%% This function plan the trajectory for the left-leg process.
%
global uLINK
ToRad = pi/180;
ToDeg = 180/pi;

%% Initialize parameters
th_inv=[];
position_inv=[];
ref_inv=[];
position_left_ankle = [];

%% Target trajectory generation(one that is generated w.r.t the right ankle)
% for storage
traj_x = [];
traj_y = [];
traj_z = [];
% Obtain the trajectory in world frame
%Pini = [0, 0, 0] - P_end;
%P_end_tmp = [0, 0, 0];
Pini = [0, 0, 0];
% Fit a particular curve, and manually set three points for the trajectory, and Fit a parabola to it.
%position_mid_point = (Pini - P_end_tmp) / 2;
%position_matrix = [Pini; position_mid_point; P_end_tmp];
position_mid_point = (Pini + P_end) / 2;
position_matrix = [Pini; position_mid_point; P_end];

x0 = position_matrix(:, 1)' + [0, -50, 0];
y0 = position_matrix(:, 2)' + [0, -50, 0];
p = polyfit(y0,x0,2);
% For each discrete point on the trajectory, calculate the minimum needed force and corresponding joint configuration
for y = Pini(2):5:P_end(2)%_tmp(2)
    % obtain corresponding x from the fitted function
    x = p(1) * y .^2 + p(2) * y .^1 + p(3);
    
    % manually set z coordinate
    height = 100;
    if y/5 <= 92 % len/2
        % first half of the trajectory
        z = height / 92 * (y/5);
    else
        % second half of the trajectory, symmetric to the first half
        z = height - height / 92 * (y/5-92);
    end
    
    traj_x = [traj_x, x];
    traj_y = [traj_y, y];
    traj_z = [traj_z, z];
    
end

% draw the trajectory in 3d space
%{
figure;
plot3(traj_x, traj_y, traj_z, 'b');
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
hold on;
plot3(Pini(1), Pini(2), Pini(3), 'rx');
plot3(P_end(1), P_end(2), P_end(3), 'rx');
plot3(traj_x, traj_y, zeros(size(traj_x)), 'r--');
plot3(0, 920, 0, 'kx');
%}

% draw the projection of the trajectory onto the X-Y plane
%{
figure();
plot(traj_x, traj_y, 'b');
grid on;
xlabel('x'); ylabel('y');
hold on;
plot(Pini(1), Pini(2), 'rx');
plot(P_end(1), P_end(2), 'rx');
plot(x0, y0, 'rx');
%}

%% Calculate the Jacobian matrix
global th ref_phase
current_joint_conf = th_init * ToRad;
current_pos = [0,0,0];
J = zeros(6, 8); % 6 DOF for the end-point, 8 DOF for movable joints
for i = 1:length(traj_x)
    % Update the robot's joint configuration for updating its pose
    th = current_joint_conf; % in radius
    ref_phase = 1; % choosing the right ankle as the base
    wholebody; % update th in radius!!! check out how SetPose() use th to update
    
    % calculate the jacobian matrix
    idx = [12, 11, 10, 8, ...
           2, 4, 5, 6];
    for j = 1:8
        a = uLINK(idx(j)).R * uLINK(idx(j)).a; % or just [1,0,0]?
        J(:, j) = [cross(a, uLINK(7).p - uLINK(idx(j)).p); a]; % error fixed...
    end
    
    % obtain differential motion
    dMov = [traj_x(i)-current_pos(1); traj_y(i)-current_pos(2); traj_z(i)-current_pos(3); 0; 0; 0];
    %invJ = inv(J'*J)*J'; % direction calculation of pseudo-inverse
    dJoint = pinv(J)*dMov; % pseudo-inverse based on SVD
    
    % record new joint configuration
    %tmp = [dJoint(1), 0, dJoint(2), dJoint(3), dJoint(4), 0,...
    %       dJoint(5), 0, dJoint(6), dJoint(7), dJoint(8), 0];
    %tmp = [dJoint(7), dJoint(8), dJoint(9), dJoint(10), dJoint(11), dJoint(12),...
    %       dJoint(6), dJoint(5), dJoint(4), dJoint(3), dJoint(2), dJoint(1)];
    tmp = [dJoint(5), 0, dJoint(6), dJoint(7), dJoint(8), 0,...
           dJoint(4), 0, dJoint(3), dJoint(2), dJoint(1), 0];
    th_inv = [th_inv; current_joint_conf + tmp]; % record target joint configuration, in radius
    
    % record the position of the base, for the update in wholebody.m
    p1 = uLINK(1).p';
    position_inv = [position_inv; p1];
    
    % record the left ankle's position
    p2 = uLINK(7).p';
    position_left_ankle = [position_left_ankle; p2];
    
    % record the phase
    ref_inv = [ref_inv; 1];
    
    % update current joint configuration
    current_joint_conf = current_joint_conf + tmp;
    
    % update current position of the left ankle
    current_pos = [traj_x(i), traj_y(i), traj_z(i)];
    %current_pos = uLINK(7).p;
    
end

th_inv = th_inv * ToDeg;


end