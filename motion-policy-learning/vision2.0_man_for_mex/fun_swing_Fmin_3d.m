function f = fun_swing_Fmin_3d(x)
%% The objective function for optimizing ducted fan's thrust with the use of ZMP calculation

%% Load needed global variable and set up some values
global ref_phase uLINK x_last
global expected_p_lx expected_p_ly expected_p_lz expected_zmp angular_acc mu_k avg_r

%% Relevant parameters
D = 100;
ToDeg = 180/pi;

%% Assign joint angles
if ref_phase == 0
    angle_rotation = atan(abs(expected_p_ly - expected_zmp(2))/abs(expected_p_lx - expected_zmp(1)));
    th_tmp = [90, 0, x(1), x(2), x(3), 0, angle_rotation * ToDeg, ...
             -90, 0, x(4), x(5), x(6), 0, 0]; % reverse angle_rotation
else % ref_phase == 1   
    angle_rotation = atan(abs(expected_p_lx - expected_zmp(1))/abs(expected_p_ly - expected_zmp(2)));
    th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
             -90, 0, x(4), x(5), x(6), 0, angle_rotation * ToDeg]; % reverse angle_rotation
end
    
%% Update the robot's state
write(th_tmp); % ref_phase has already been set in optimize_joint_angles.m


%% New model considering the friction torque
% the position of the left ducted fan
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

%% Set up nonlinear inequality constraint on the left ducted fan's thrust
% required thrust
ankle_theta = atan( ...
                  (mu_k * avg_r * g * (-r_Ly + D + R_cy) + angular_acc * R_cy ^2 * (-r_Ly + D))/ ...
                  ((r_Ly - D) * g * R_cy) ...
                  );
           
f = M * g * R_cy / ((r_Ly - D) * cos(ankle_theta));


%% Display information
% ['thrust = ', num2str(f)]


%% Add other objectives
% f = f + norm(p_l - [expected_p_lx, expected_p_ly, expected_p_lz]) + ...
%      norm(x - x_last);%+ norm(r_ly / r_lx - r_Gy / r_Gx)*50;


end
