function [c, ceq] = Position_swing_con_based_on_zmp(x)
%% This function computes the nonlinear constraints.

%% Set up needed parameters

D = 100;
H = 40;

c = [];
ceq = [];

%% Load needed global variable and set up some values
global expected_p_lx expected_p_ly expected_p_lz ref_phase uLINK expected_zmp mu_k avg_r angular_acc

% initialize
th_tmp = [90, 0, x(1), x(2), x(3), 0, 0, ...
         -90, 0, x(4), x(5), x(6), 0, 0]; % without the rotation of the left ankle

ref_phase = 1;

%% Update the robot's state
write(th_tmp);

%% Calculate needed parameters
% the position of the swing leg's ankle, in 3-dim space
p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
p_zmp = expected_zmp;
r_L = p_L - p_zmp;
r_Ly = r_L(2); r_Lz = r_L(3);

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

%% Set up nonlinear equality constraint
%% l - left ducted fan; L - left ankle
%% r - 3-dim; R - 2-dim (when the angle of rotation of the support lef is set to 0, r_Ly == R_Ly)
% planned trajectory of the left [ankle], in 3-dim space
expected_r_lx = expected_p_lx - p_zmp(1); 
expected_r_ly = expected_p_ly - p_zmp(2); 
expected_r_lz = expected_p_lz - p_zmp(3);
% planned trajectory of the left [ankle], in 2-dim frame
expected_R_Ly = -(expected_r_lx^2+expected_r_ly^2)^0.5;
expected_R_Lz = expected_r_lz;

% actual trajectory of the left ankle, in 2-dim frame
R_Ly = r_Ly; % left ankle's position % now r_Ly == R_Ly %-(r_Ly^2+r_Lx^2)^0.5; 
R_Lz = r_Lz;

ceq = [ceq; ...
       expected_R_Ly - R_Ly;...
       expected_R_Lz - R_Lz];

%% Set up nonlinear inequality constraint on the left ducted fan's thrust
% required thrust
ankle_theta = atan( ...
                  (mu_k * avg_r * g * (-r_Ly + D + R_cy) + angular_acc * R_cy ^2 * (-r_Ly + D))/ ...
                  ((r_Ly - D) * g * R_cy) ...
                  );
           
f = M * g * R_cy / ((r_Ly - D) * cos(ankle_theta));

c = [c; f - 23]; % the maximum force is 20

%% Display message
% ['ceq = [', num2str(ceq'), ']; c = [', num2str(c), '].']
% ['c_traj = [', num2str(c(1:3)'), '], c_cross = ', num2str(c(4))]
% ['c_traj = [', num2str(c(1:3)'), '].']


%% Check out the constraints
%{
figure;
plot([r_l(1), r_G(1)], [r_l(2), r_G(2)], 'b-');
k = (r_l(2) - r_G(2)) / (r_l(1) - r_G(1));
x = 0;
y = k * (x - r_l(1)) + r_l(2);
hold on;
plot([r_G(1), x], [r_G(2), y], 'b-');
plot(0, 0, 'rx');
grid on;
xlabel('x'); ylabel('y');
%}

end