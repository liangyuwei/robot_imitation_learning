function [q, f] = FindMinF_lift_3d(pivot)
%% This function optimizes the required force for the ducted fan to exert, based on ZMP calculation.
% There are two feasible schemes for making use of the ZMP criterion:
% 1. Manually set the [range] of ZMP position, and apply optimization with it;(ongoing) 
% 2. Manually set the [coordinate] of the CoM, and apply optimization with it.

global x_last expected_zmp

%% Initialize relevent parameters    
x0 = x_last; 
expected_zmp = pivot;

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
[x, fval] = fmincon(@fun_lift_Fmin_3d, x0, A, b, Aeq, beq, lb, ub, @Position_lift_con_3d); % check nonlienar constraints
f = fval;

%% return the optimization result
q = [90, 0, x(1), x(2), x(3), 0, 0, ...
    -90, 0, x(4), x(5), x(6), 0, 0];

% record the current joint configuration
x_last = x;
                                                    
end
