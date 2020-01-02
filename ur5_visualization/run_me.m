% File              : run_me.m
% Author            : Pradeep Rajendran <pradeepunique1989@gmail.com>
% Date              : 06.10.2018
% Last Modified Date: 29.01.2019
% Last Modified By  : Pradeep Rajendran <pradeepunique1989@gmail.com>

addpath(genpath(get_this_dir()));
%% COMPILE MEX CODE
%{
cd ur_kinematics;
run compile_ur5kinematics.m
cd ..
%}

%% BOILERPLATE
f = figure();
axis equal;
grid on;
grid minor;
camlight;
material metal;
axis([-0.6 0.6 -0.6 0.6 0 1]*2);

global uLINK th
th = zeros(6, 1);
UX = [1 0 0]'; UY = [0 1 0]'; UZ = [0 0 1]';
uLINK = struct('name','body', 'mom', 0, 'child', 2, 'b', [0 0 0]', 'a', UZ, 'q', 0);
uLINK(2) = struct('name', 'shoulder_link', 'mom', 1, 'child', 3, 'b', [0 0 0.089159]', 'a', UZ, 'q', th(1));
uLINK(3) = struct('name', 'upper_arm_link', 'mom', 2, 'child', 4, 'b', [0  0.13585 0]',  'a', UY, 'q', th(2));
uLINK(4) = struct('name', 'forearm_link', 'mom', 3, 'child', 5, 'b', [0.425 -0.1197 0]', 'a', UY,'q', th(3));
uLINK(5) = struct('name', 'wrist_1_link', 'mom', 4, 'child', 6, 'b', [0.39225 0 0]' , 'a', UY, 'q', th(4));
uLINK(6) = struct('name', 'wrist_2_link', 'mom', 5, 'child', 7, 'b', [0 0.093 0]' ,'a', -UZ, 'q', th(5));
uLINK(7) = struct('name', 'wrist_3_link', 'mom', 6, 'child', 8, 'b', [0 0 -0.09465]' ,'a', UY, 'q', th(6)); % child = 0 indicates that the calculation of forward kinematics should stop here
uLINK(8) = struct('name', 'ee_link', 'mom', 7, 'child', 0, 'b', [0 0.0823 0]', 'a', UY, 'q', pi/2); % eef, if necessary

uLINK(1).R = eye(3);    
uLINK(1).p = [0, 0, 0.4]';

ur5_disp = UR5Display(f);


%% MOVING TO A GIVEN CONFIGURATION
ur5_disp.draw_configuration([0,0,0,0,0,0]);%([pi/4, -pi/4, pi/2, pi/6, pi/3, pi/4]); %([pi/4, -pi/4, pi/2, 0, 0, 0]); %([-0.95 -0.85 -1.2 0.95 0.95 0.6]);
xlabel('x');
ylabel('y');
zlabel('z');

%% KEYBOARD CONTROL
%{
fprintf("\n*************** KEYBOARD CONTROL **********************\n");
fprintf("Press the following keys for joint motion\n");
fprintf("q,w,e,r,t,y for incrementing joint angles 1 through 6\n");
fprintf("a,s,d,f,g,h for decrementing joint angles 1 through 6\n");
fprintf("u for increasing the granularity of joint angle change\n");
fprintf("j for decreasing the granularity of joint angle change\n");
fprintf("\n***********************************************\n");
 %}

%% FORWARD AND INVERSE KINEMATICS
%{
% Forward kinematics
ur5_kin = UR5Kinematics();
result = ur5_kin.forward_kinematics([0 0 0 0 0 0]); % obtains the transform frames of all links
result.transform_matrices

% Inverse kinematics
T_ee = eye(4);
T_ee(1:3, 1:3) = eye(3); % this sets the rotation component of the end-effector
T_ee(1:3, 4) = [0.6; 0.1; 0.6]; % this sets the position component of the end-effector
[ik_sols] = ur5_kin.inverse_kinematics(T_ee, 0); % computes the IK solutions to reach end-effector configuration T_ee

ur5_disp.draw_configuration(ik_sols(2, :)); % visualize one of the solutions
%}