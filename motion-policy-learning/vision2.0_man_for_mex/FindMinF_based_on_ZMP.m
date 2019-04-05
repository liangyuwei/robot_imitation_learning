function [q, f] = FindMinF_based_on_ZMP
%% This function optimizes the required force for the ducted fan to exert, based on ZMP calculation.
% There are two feasible schemes for making use of the ZMP criterion:
% 1. Manually set the [range] of ZMP position, and apply optimization with it;(ongoing) 
% 2. Manually set the [coordinate] of the CoM, and apply optimization with it.

ToDeg = 180 / pi;
D = 100;

global x_last expected_zmp avg_r mu_k angular_acc expected_p_lx expected_p_ly uLINK
% ['left leg', 'right leg', 'hips', 'right foot']
% NEW - ['left leg', 'right leg']

%% Initialize relevent parameters
q = [];     

% initial value
% x0 = [90;0;-90; 90;0;-90]; % in radius
x0 = x_last; 
% x0 = [ 90;0;-90;  51.78; -81.78; 30];

% constraints
A = []; b = []; 
Aeq = [1 1 1 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 1 1;
       0 0 0 0 0 0;
       0 0 0 0 0 0]; % keep the base horizontal to the ground
beq=[0 0 0 0 0 0]';
  
% lower and upper bound
% lb=[-30;
%     -120;
%     -100;
%     
%     -30;
%     -150;
%     -100];
lb=[-48;
    -130;
    -100;
    
    -48;
    -130;
    -100]; % physical limits
% 90 0 -90 90 0 -90 / 90 -90
% ub=[100;
%     90;
%     120;
%     
%     100;
%     90;
%     120]; 
ub=[115;
    150;
    60;
    
    115;
    150;
    60]; % physical limits 

%% Start optimization
% Treat it as a one-dimensional optimization problem as before, and
% calculate ankle3's angles as well as the thrust directly through the formula!!! 
[x, fval] = fmincon(@fun_swing_Fmin_based_on_zmp, x0, A, b, Aeq, beq, lb, ub, @Position_swing_con_based_on_zmp); % check nonlienar constraints

%% Calculate the angles of RfootCenter(th(14)) as well as Lankle1(th(6)) directly using the formula
% % calculate the angle of rotation of the whole robot
angle_rotation = atan(abs(expected_p_lx - expected_zmp(1))/abs(expected_p_ly - expected_zmp(2)));

q_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
        -90, 0, x(4), x(5), x(6), 0, 0]; % using the optimization result without regarding the rotation of the left foot.

% update the robot's state
write(q_tmp);

% calculate r_l, i.e. the position of the left ducted fan!!!
p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
p_zmp = expected_zmp;
r_L = p_L - p_zmp;
r_Ly = r_L(2);

% CoM of the whole robot without the part of the swing leg's foot
mc = 0; M = 0;
for i = 1:15
    mc = mc + uLINK(i).c * uLINK(i).m;        
    M = M + uLINK(i).m;
end
p_c = mc ./ M;
r_c = p_c' - p_zmp;
R_cy = r_c(2);


g = 9.8; % gravitational acceleration

%% Set up nonlinear inequality constraint on the left ducted fan's thrust
% required thrust
ankle_theta = atan( ...
                  (mu_k * avg_r * g * (-r_Ly + D + R_cy) + angular_acc * R_cy ^2 * (-r_Ly + D))/ ...
                  ((r_Ly - D) * g * R_cy) ...
                  );
           
f = fval; % check by this % M * g * R_cy / ((r_Ly - D) * cos(ankle_theta));

%% return the optimization result
q = [90, 0, x(1), x(2), x(3), -ankle_theta * ToDeg, 0, ...
    -90, 0, x(4), x(5), x(6), 0, -angle_rotation * ToDeg];

% record the current joint configuration
x_last = x;
                                                    
end
