function [c, ceq] = Position_lift_con_3d(x)
%% This function computes the nonlinear constraints.

%% Set up needed parameters

D = 100;

c = [];
ceq = [];

%% Load needed global variable and set up some values
global expected_p_lx expected_p_ly expected_p_lz ref_phase uLINK expected_zmp mu_k avg_r angular_acc det

% initialize
th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
         -90, 0, x(4), x(5), x(6), 0, 0]; % without the rotation of the left ankle

%% Update the robot's state
write(th_tmp); % ref_phase is already set in optimize_joint_angles.m

%% Calculate needed parameters
if ref_phase == 0 
    % left foot as the pivot, execute the RIGHT ANKLE2's trajectory
    p_L = uLINK(14).p;
else
    % right foot as the pivot, execute the LEFT ANKLE2's trajectory
    p_L = uLINK(7).p;
end
p_zmp = expected_zmp;
arm_thrust = norm([p_L(1) - p_zmp(1), p_L(2) - p_zmp(2)]) + D;

% CoM of the whole robot without the part of the swing leg's foot
mc = 0; M = 0;
for i = 1:15
    mc = mc + uLINK(i).c * uLINK(i).m;        
    M = M + uLINK(i).m;
end
p_c = mc ./ M;
arm_gravity = sign(p_c(2) - p_zmp(2)) *  norm([p_c(1) - p_zmp(1), p_c(2) - p_zmp(2)]);

g = 9.8; % gravitational acceleration

%% Set up nonlinear equality constraint
ceq = [norm(p_L(1:2)'-p_zmp(1:2)) - norm([expected_p_lx, expected_p_ly]-p_zmp(1:2));...
       p_L(3) - expected_p_lz];
   
   
%% tmp - calculate the gravitational moment through geometric method
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

% f = M * g * arm_gravity / arm_thrust;
f = 9.8 * Lcom / yend;

% c = [c; f - 23]; % the maximum force is 20
% c = [c; -f; f - 23];
c = [c; -f];

%% Display message
% ['ceq = [', num2str(ceq'), ']; c = [', num2str(c'), '].']
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