function [c, ceq] = Position_swing_con_3d(x)
%% This function computes the nonlinear constraints.

%% Set up needed parameters
ToDeg = 180/pi;
D = 100;
H = 40;

c = [];

%% Load needed global variable and set up some values
global expected_p_lx expected_p_ly expected_p_lz ref_phase uLINK expected_zmp mu_k avg_r angular_acc

% initialize
if ref_phase == 0
    angle_rotation = atan(abs(expected_p_ly - expected_zmp(2))/abs(expected_p_lx - expected_zmp(1)));
    th_tmp = [90, 0, x(1), x(2), x(3), 0, angle_rotation * ToDeg, ...
             -90, 0, x(4), x(5), x(6), 0, 0]; % reverse angle_rotation
else    
    angle_rotation = atan(abs(expected_p_lx - expected_zmp(1))/abs(expected_p_ly - expected_zmp(2)));
    th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
             -90, 0, x(4), x(5), x(6), 0, angle_rotation * ToDeg]; % reverse angle_rotation
end
%% Update the robot's state
write(th_tmp); % ref_phase has already been set in optimize_joint_angles.m

%% Calculate needed parameters
% the position of the swing leg's ankle, in 3-dim space
if ref_phase == 0 
    % left foot as the pivot, execute the RIGHT ANKLE2's trajectory
    p_L = uLINK(14).p;
else
    % right foot as the pivot, execute the LEFT ANKLE2's trajectory
    p_L = uLINK(7).p;
end
p_zmp = expected_zmp; % expected_zmp is already set to be left_pivot or right_pivot
arm_thrust = norm([p_L(1) - p_zmp(1), p_L(2) - p_zmp(2)]) + D;
r_Ly = - arm_thrust;

% CoM of the whole robot without the part of the swing leg's foot
mc = 0; M = 0;
for i = 1:15
    mc = mc + uLINK(i).c * uLINK(i).m;        
    M = M + uLINK(i).m;
end
p_c = mc ./ M;
arm_gravity = norm([p_c(1) - p_zmp(1), p_c(2) - p_zmp(2)]);
R_cy = - arm_gravity;


g = 9.8; % gravitational acceleration

%% Set up nonlinear equality constraint
ceq = [expected_p_lx - p_L(1); ...
       expected_p_ly - p_L(2);...
       expected_p_lz - p_L(3)];


%% Set up nonlinear inequality constraint on the left ducted fan's thrust
% required thrust
ankle_theta = atan( ...
                  (mu_k * avg_r * g * (-r_Ly + D + R_cy) + angular_acc * R_cy ^2 * (-r_Ly + D))/ ...
                  ((r_Ly - D) * g * R_cy) ...
                  );
           
f = M * g * R_cy / ((r_Ly - D) * cos(ankle_theta));

c = [c; f - 23]; % the maximum force is 20

%% Display message
% ['ceq = [', num2str(ceq'), ']; c = [', num2str(c), '].']
% ['c_traj = [', num2str(c(1:3)'), '], c_cross = ', num2str(c(4))]
% ['c_traj = [', num2str(c(1:3)'), '].']


%% Check out the constraints
%{
figure;
plot([r_l(1), r_G(1)], [r_l(2), r_G(2)], 'b-');
k = (r_l(2) - r_G(2)) / (r_l(1) - r_G(1));
x = 0;
y = k * (x - r_l(1)) + r_l(2);
hold on;
plot([r_G(1), x], [r_G(2), y], 'b-');
plot(0, 0, 'rx');
grid on;
xlabel('x'); ylabel('y');
%}

end