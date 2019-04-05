function [q, f] = FindMinF_swing_3d(pivot)
%% This function optimizes the required force for the ducted fan to exert, based on ZMP calculation.
% There are two feasible schemes for making use of the ZMP criterion:
% 1. Manually set the [range] of ZMP position, and apply optimization with it;(ongoing) 
% 2. Manually set the [coordinate] of the CoM, and apply optimization with it.

ToDeg = 180 / pi;
D = 100;

global x_last expected_zmp avg_r mu_k angular_acc expected_p_lx expected_p_ly uLINK ref_phase
% ['left leg', 'right leg', 'hips', 'right foot']
% NEW - ['left leg', 'right leg']

%% Initialize relevent parameters
expected_zmp = pivot;   

% initial value
x0 = x_last;

% constraints
A = []; b = []; 
Aeq = [1 1 1 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 1 1;
       0 0 0 0 0 0;
       0 0 0 0 0 0]; % keep the base horizontal to the ground
beq=[0 0 0 0 0 0]';
  
% lower and upper bounds
lb=[-48;
    -130;
    -100;
    
    -48;
    -130;
    -100]; % physical limits
ub=[115;
    150;
    60;
    
    115;
    150;
    60]; % physical limits 


%% Start optimization
[x, fval] = fmincon(@fun_swing_Fmin_3d, x0, A, b, Aeq, beq, lb, ub, @Position_swing_con_3d); % check nonlienar constraints

%% Calculate the angles of RfootCenter(th(14)) as well as Lankle1(th(6)) directly using the formula
% % calculate the angle of rotation of the whole robot
if ref_phase == 0
    angle_rotation = atan(abs(expected_p_ly - expected_zmp(2))/abs(expected_p_lx - expected_zmp(1)));
    q_tmp = [90, 0, x(1), x(2), x(3), 0, angle_rotation * ToDeg, ...
             -90, 0, x(4), x(5), x(6), 0, 0]; % reverse angle_rotation
else    
    angle_rotation = atan(abs(expected_p_lx - expected_zmp(1))/abs(expected_p_ly - expected_zmp(2)));
    q_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
             -90, 0, x(4), x(5), x(6), 0, angle_rotation * ToDeg]; % reverse angle_rotation
end
% update the robot's state
write(q_tmp);

% calculate r_l, i.e. the position of the left ducted fan!!!
if ref_phase == 0 
    % left foot as the pivot, execute the RIGHT ANKLE2's trajectory
    p_L = uLINK(14).p;
else
    % right foot as the pivot, execute the LEFT ANKLE2's trajectory
    p_L = uLINK(7).p;
end
p_zmp = expected_zmp;
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
           
f = fval;

%% return the optimization result
if ref_phase == 0
    angle_rotation = atan(abs(expected_p_ly - expected_zmp(2))/abs(expected_p_lx - expected_zmp(1)));
    q = [90, 0, x(1), x(2), x(3), 0, angle_rotation * ToDeg, ...
        -90, 0, x(4), x(5), x(6), ankle_theta * ToDeg, 0];
else    
    angle_rotation = atan(abs(expected_p_lx - expected_zmp(1))/abs(expected_p_ly - expected_zmp(2)));
    q = [90, 0, x(1), x(2), x(3), ankle_theta * ToDeg, 0, ...
        -90, 0, x(4), x(5), x(6), 0, angle_rotation * ToDeg];     
end

% record the current joint configuration
x_last = x;
                                                    
end
