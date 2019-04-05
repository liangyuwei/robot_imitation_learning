function [c1, ceq] = Position_con(x)

% target position
global P_end det

%% Equalities constraints(Ceq(X)==0)
% what about det??? 
% the former part is the horizontal distance between two
% ankles(uLINK(7)&uLINK(13)), and the latter part is the current point on
% the trajectory. Thus (yend==0) means (dist_ankles==P_end(2)), i.e. right 
% ankle is on the trajectory.
yend=(220*sin(x(1))+140*sin(x(3))+200+220*sin(x(6))+140*sin(x(4))-P_end(2));
zend=(220*cos(x(1))+140*cos(x(3))-220*cos(x(6))-140*cos(x(4))-P_end(3));
ceq=[yend;zend];

%% Equalities constraint(C(X)<=0)
c1=[];

end
