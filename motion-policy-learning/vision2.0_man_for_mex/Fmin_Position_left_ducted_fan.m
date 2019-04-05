function [q, fval] = Fmin_Position_left_ducted_fan(y, z)
%% This function calculates the minimum required force for every point on the trajectory

%% Initialize parameters
global P_end x_last
P_end = [0,y,z]; % assign each point location on the trajectory for optimization
% P_end is the position of right ankle w.r.t the left ankle coordinate.

% initial values
% x0=[90;0;-90; 90;0;-90]; % in degree
x0 = x_last;

% inequalities constraints
A=[];b=[];

% equalities constraints
Aeq=[1 1 1 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 1 1 1 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0];
beq=[0 0 0 0 0 0]';

% variables' bound
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
    60];

% start optimization
[x, fval] = fmincon(@fun_Fmin_left_ducted_fan,x0,A,b,Aeq,beq,lb,ub,@Position_con_left_ducted_fan);

% return results
q = [90, 0, x(1), x(2), x(3), 0, 0, ...
    -90, 0, x(4), x(5), x(6), 0, 0];

% record the current result for later use
x_last = x;

end

