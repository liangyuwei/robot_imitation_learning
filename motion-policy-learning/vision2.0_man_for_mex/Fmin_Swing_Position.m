function [q,Fmin]=Fmin_Swing_Position(x,y,z)
%% This function calculates the minimum required force for every point on the trajectory

%% Initialize parameters
global P_end;
P_end=[x, y, z]; % assign to P_end each point location on the trajectory for optimization
% P_end is the position of right ankle w.r.t the initial position of the left ankle.(world frame)

% initial values(add 2 joints - hips)
x0=[0;0;0;0;0;0;0;0].*pi/180;

% inequalities constraints
A=[];b=[];

% equalities constraints
Aeq=[1 1 -1 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 1 1 -1 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0]; % keep the base horizontal to the ground
beq=[0 0 0 0 0 0 0 0]';

% variables' bound
lb=[-50;
    -150;
    -0;
    -0;
    -00;
    -50;
    -90;
    -100].*pi/180;
ub=[100;
    130;
    115;
    115;
    130;
    100;
    90;
    0].*pi/180; % hip joint angle range within [-150,150]

% start optimization
%[x, fval] = fmincon(@fun_swing_Fmin,x0,A,b,Aeq,beq,lb,ub,@Position_swing_con);
[x, fval] = fmincon(@fun_swing_Fmin,x0,A,b,Aeq,beq,lb,ub,@Position_swing_con);

% return results
q = [x(7), 0, x(3), x(2), x(1), 0, ...
     x(8), 0, x(4), x(5), x(6), 0] * 180/pi;

Fmin = fval;

end

