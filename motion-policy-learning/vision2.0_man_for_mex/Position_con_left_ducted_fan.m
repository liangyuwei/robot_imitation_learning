function [c1, ceq] = Position_con_left_ducted_fan(x)

% target position
global P_end det Pend uLINK expected_zmp

% Get the joint configuration and use it to update the robot's pose
th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
         -90, 0, x(4), x(5), x(6), 0, 0];
write(th_tmp);

%% Equalities constraints(Ceq(X)==0)
p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)]; % the pos of the left ankle
ceq = [p_L(2) - P_end(2); p_L(3) - 40 - P_end(3)]; % joint_feet_displacement = [0, 0, -40];
% yend = Pend(2) - (220*sin(x(1))+140*sin(-x(3))+200+220*sin(-x(6))+140*sin(x(4))) - P_end(2);
% zend = Pend(3) - (220*cos(x(1))+140*cos(-x(3))-220*cos(-x(6))-140*cos(x(4))) - P_end(3);
% ceq = [yend;zend];

%% Equalities constraint(C(X)<=0)
% the arm of the left ducted fan's thrust
p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
p_zmp = expected_zmp;
r_L = p_L - p_zmp;
r_Ly = r_L(2);
% the arm of the gravity
mc = 0; M = 0;
for i = 1:15
    mc = mc + uLINK(i).c * uLINK(i).m;        
    M = M + uLINK(i).m;
end
p_c = mc ./ M;
r_c = p_c' - p_zmp;
R_cy = r_c(2);
% calculate the needed thrust from the left ducted fan
g = 9.8; D = 100;
f = M * g * abs(R_cy) / (abs(r_Ly)+D);

% c1 = [-f; f - 23];
c1 = [];

%% Display the message
% ['ceq = [', num2str(ceq'), '], c1 = [', num2str(c1'), '].']

end
