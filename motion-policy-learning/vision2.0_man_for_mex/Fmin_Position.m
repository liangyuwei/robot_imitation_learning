function [q,Fmin]=Fmin_Position(y,z)
%% This function calculates the minimum required force for every point on the trajectory

%% Initialize parameters
global P_end;
P_end=[0,y,z]; % assign each point location on the trajectory for optimization
% P_end is the position of right ankle w.r.t the left ankle coordinate.

% initial values
x0=[0;0;0;0;0;0].*pi/180;
%x0 = x_last.*pi/180;

% inequalities constraints
A=[];b=[];

% equalities constraints
Aeq=[1 1 -1 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 1 1 -1 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0];
beq=[0 0 0 0 0 0]';

% variables' bound
lb=[0;%-50
    -150;
    -0;
    -0;
    -00;
    -50
    ].*pi/180;
ub=[100;
    130;
    115;
    115;
    130;
    100].*pi/180;

% start optimization
[x, fval] = fmincon(@fun_Fmin,x0,A,b,Aeq,beq,lb,ub,@Position_con);

% return the current joint configurations
%x_cur = x;

% return results
q = [pi/2, 0, x(3), x(2), x(1), 0, ...
    -pi/2, 0, x(4), x(5), x(6), 0] * 180/pi;

Fmin = fval;

end

