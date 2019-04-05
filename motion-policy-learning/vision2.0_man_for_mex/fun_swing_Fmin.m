function f = fun_swing_Fmin(x)
%% This function should be adapted to swinging phase(left-leg phase).
global det

%% The distance between the left ducted fan and the right heel.(Slightly Different from the right-leg phase!!!!)
% using the right heel as a pivot to derive the equation of moment equilibrium
tmp1 = 220*sin(x(1))+140*sin(x(3))+100; tmp2 = 200*cos(x(8)+pi/2) + 220*sin(x(6))+140*sin(x(4))+100+det;
q1 = x(7); q2 = -x(8)-pi/2;
dist = sqrt((tmp1*cos(q1) + tmp2*sin(q2))^2 + (tmp1*sin(q1) + 200 + tmp2*cos(q2))^2);

% the y coordinate of CoM, choosing the right heel as the ZMP
% mirror ??? same mass, same length
Lcom = (0.632*(220*sin(x(1))+140*sin(x(3))+200+220*sin(x(6))+140*sin(x(4))+200+det))...
    +(0.718*(220*sin(x(1))+140*sin(x(3))+200+220*sin(x(6))+140*sin(x(4))+100+det))...
    +(0.667*(220*sin(x(1))+140*sin(x(3))+200+140*sin(x(4))+100+det))...
    +(0.645*(220*sin(x(1))+140*sin(x(3))+200+100+det))...
    +(0.300*(220*sin(x(1))+140*sin(x(3))+100+100+det))...
    +(0.645*(220*sin(x(1))+140*sin(x(3))+100+det))...
    +(0.667*(220*sin(x(1))+100+det)) ...
    +(1.35*100+det);

% calculate the required force
f = 9.8 * Lcom / dist; %(goal is to minimize f through optimization)

end

