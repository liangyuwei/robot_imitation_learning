function f = fun_Fmin_left_ducted_fan(x)
global det expected_zmp uLINK

D = 100;
g = 9.8;

% Get the joint configuration and use it to update the robot's pose
th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
         -90, 0, x(4), x(5), x(6), 0, 0];
write(th_tmp);
     
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
f = M * g * abs(R_cy) / (abs(r_Ly)+D);

end