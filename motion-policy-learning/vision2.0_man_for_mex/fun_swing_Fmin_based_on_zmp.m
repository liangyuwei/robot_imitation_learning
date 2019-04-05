function f = fun_swing_Fmin_based_on_zmp(x)
%% The objective function for optimizing ducted fan's thrust with the use of ZMP calculation

%% Load needed global variable and set up some values
global ref_phase uLINK x_last
global expected_p_lx expected_p_ly expected_p_lz expected_zmp angular_acc mu_k avg_r

%% Relevant parameters
D = 100;

%% Assign joint angles
th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
         -90, 0, x(4), x(5), x(6), 0, 0]; % reverse angle_rotation
     
ref_phase = 1;


%% Update the robot's state
write(th_tmp);


%% New model considering the friction torque
% the position of the left ducted fan
p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
p_zmp = expected_zmp;
r_L = p_L - p_zmp;
r_Ly = r_L(2);

% CoM of the whole robot without the part of the swing leg's foot
mc = 0; M = 0;
for i = 1:15
    mc = mc + uLINK(i).c * uLINK(i).m;        
    M = M + uLINK(i).m;
end
p_c = mc ./ M;
r_c = p_c' - p_zmp;
R_cy = r_c(2);


g = 9.8; % gravitational acceleration

%% Set up nonlinear inequality constraint on the left ducted fan's thrust
% required thrust
ankle_theta = atan( ...
                  (mu_k * avg_r * g * (-r_Ly + D + R_cy) + angular_acc * R_cy ^2 * (-r_Ly + D))/ ...
                  ((r_Ly - D) * g * R_cy) ...
                  );
           
f = M * g * R_cy / ((r_Ly - D) * cos(ankle_theta));


%% Display information
% ['thrust = ', num2str(f)]


%% Add other objectives
% f = f + norm(p_l - [expected_p_lx, expected_p_ly, expected_p_lz]) + ...
%      norm(x - x_last);%+ norm(r_ly / r_lx - r_Gy / r_Gx)*50;


end
