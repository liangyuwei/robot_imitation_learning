function [c1, ceq] = Position_swing_con(x)

% target position
global P_end det

%% Equalities constraints(Ceq(X)==0)
% what about det??? 
% the former part is the horizontal distance between two
% ankles(uLINK(7)&uLINK(13)), and the latter part is the current point on
% the trajectory. Thus (yend==0) means (dist_ankles==P_end(2)), i.e. right 
% ankle is on the trajectory.
xend = -(220*sin(x(1))+140*sin(x(3)))*sin(pi+x(8)-x(7)) - 200*sin(x(8)+pi/2) - P_end(1);
yend = (220*sin(x(1))+140*sin(x(3)))*cos(pi+x(8)-x(7)) + 200*cos(x(8)+pi/2) + 220*sin(x(6))+140*sin(x(4)) - P_end(2);
zend = (220*cos(x(1))+140*cos(x(3))-220*cos(x(6))-140*cos(x(4)) - P_end(3));
ceq=[xend;yend;zend];

% 2018-01-31:
% should add constraints on static balance(embeded into fun_swing_Fmin.m
% the objective function???) and force acting position(ducted-fan's thrust, gravity should be in the same plane)


%% Equalities constraint(C(X)<=0)
c1=[];

end
