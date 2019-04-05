function f=fun(x)
%% This function serves as the goal function for optimization.

% ???
global det

% yend - the distance between the left heel and the right ducted-fan
% x2, x5 is not needed
yend = (220*sin(x(1))+140*sin(x(3))+200+220*sin(x(6))+140*sin(x(4))+200+det);

%{
Lcom = (0.632*(220*sin(x(1))+140*sin(x(3))+200+220*sin(x(6))+140*sin(x(4))+200)...
    +(0.718*(220*sin(x(1))+140*sin(x(3))+200+220*sin(x(6))+140*sin(x(4))+100))...
    +(0.667*(220*sin(x(1))+140*sin(x(3))+200+140*sin(x(4))+100))...
    +(0.645*(220*sin(x(1))+140*sin(x(3))+200+100))...
    +(0.300*(220*sin(x(1))+140*sin(x(3))+100+100))...
    +(0.645*(220*sin(x(1))+140*sin(x(3))+100))...
    +(0.667*(220*sin(x(1))+100)) ...
    +(1.35*100))/6.22;
%}

f = - yend;

end