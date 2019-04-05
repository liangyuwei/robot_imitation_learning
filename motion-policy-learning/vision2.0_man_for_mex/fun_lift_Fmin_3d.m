function f = fun_lift_Fmin_3d(x)
%% The objective function for optimizing ducted fan's thrust with the use of ZMP calculation

%% Load needed global variable and set up some values
global ref_phase uLINK x_last det
global expected_p_lx expected_p_ly expected_p_lz expected_zmp angular_acc mu_k avg_r

%% Relevant parameters
D = 100;

%% Assign joint angles and update the robot's configuration
th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
         -90, 0, x(4), x(5), x(6), 0, 0]; % reverse angle_rotation

%% Update the robot's state
write(th_tmp); % ref_phase is already set in optimize_joint_angles.m

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

% CoM of the whole robot without the part of the swing leg's foot
mc = 0; M = 0;
for i = 1:15
    mc = mc + uLINK(i).c * uLINK(i).m;        
    M = M + uLINK(i).m;
end
p_c = mc ./ M;
arm_gravity = sign(p_c(2) - p_zmp(2)) * norm([p_c(1) - p_zmp(1), p_c(2) - p_zmp(2)]);

% testing: using geometric method to calculate the gravitational moment
xx = x * pi/180;
if ref_phase == 1
    % right foot as the pivot, execute the LEFT ANKLE2's trajectory
    % should exchange the joint angles of the both legs
    tmp = xx(1:3);
    xx(1:3) = xx(4:6);
    xx(4:6) = tmp;
end
yend = (220*sin(-xx(3))+140*sin(xx(1))+200+220*sin(-xx(6))+140*sin(xx(4))+200+det+110);
Lcom = (0.632*(220*sin(-xx(3))+140*sin(xx(1))+200+220*sin(-xx(6))+140*sin(xx(4))+200+det+110))...
    +(0.718*(220*sin(-xx(3))+140*sin(xx(1))+200+220*sin(-xx(6))+140*sin(xx(4))+100+det+110))...
    +(0.667*(220*sin(-xx(3))+140*sin(xx(1))+200+140*sin(xx(4))+100+det+110))...
    +(0.645*(220*sin(-xx(3))+140*sin(xx(1))+200+100+det+110))...
    +(0.300*(220*sin(-xx(3))+140*sin(xx(1))+100+100+det+110))...
    +(0.645*(220*sin(-xx(3))+140*sin(xx(1))+100+det+110))...
    +(0.667*(220*sin(-xx(3))+100+det+110)) ...
    +(1.35*100+det+110);

g = 9.8; % gravitational acceleration

%% Set up nonlinear inequality constraint on the left ducted fan's thrust
% required thrust
% f = M * g * arm_gravity / arm_thrust;
f = g * Lcom / yend;

%% Display information
% ['thrust = ', num2str(f)]

end
