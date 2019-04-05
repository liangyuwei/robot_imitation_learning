function output = crossing_obstacles_with_swing(swing_radius, swing_height, mu_kinematic, time_spent_r_swing, time_spent_l_swing)
%% This function outputs a time profile of joint angles, taking the trajectory requirements as well as environment settings as input.
% Output: 'the_tot' is a time profile of joint angles, 'f_tot' is a time profile of ducted fan's thrust 
% Input: traj_setting(struct), motion_req(struct), envionment_setting


%% Assign parameters
traj_setting.end_pos_right_ankle = [0, 920, 0];
traj_setting.swing_height = swing_height; % 300;
traj_setting.swing_radius = swing_radius; % 380;

motion_req.T_tot_period = [5, time_spent_r_swing, 5, 5, time_spent_l_swing, 5];
motion_req.update_freq = 10; % Hertz

envionment_setting.mu_k = mu_kinematic; %0.3;

motion_req.rotation_angle_swing = pi/2;


%% Initialize parameters and process the imported parameters
% params related to the robot
global th uLINK  pos pl pr info1 det H_1 Torque_all Force ref_phase 
th = zeros(14,1); % joint angles
pl = [];
pr = [];
ref_phase = 0; % initialize
info1 = [];
det = -190; % distance from the ducted-fan to the heel
H_1 = 0;
Force = [];
Torque_all = [];

% params related to the trajectory and vironment
global Pend com_traj left_end_traj right_end_traj base_traj com_main_traj expected_zmp T traj_height traj_radius avg_r
global left_pivot right_pivot joint_feet_displacement full_length
full_length = 920; % the distance between the two ankles under split
joint_feet_displacement = 100;
left_pivot = [-120, 0, 0];
right_pivot = [-120, 940, 0];
pos = [0, 100, 500]'; % the position of the base w.r.t the world frame
expected_zmp = traj_setting.end_pos_right_ankle + [0, 20, 0]; % [0, 940, 0]; % the CENTER of the right foot!!!
Pend = traj_setting.end_pos_right_ankle; %[0, 920, 0];
traj_height = traj_setting.swing_height; % 300; %40 1+ 220 * cos(the_trans_end(12)*ToRad) + 140 * cos(the_trans_end(10)) - 40;
traj_radius = traj_setting.swing_radius; % 380; % the extreme position(580) % 200 + (140 + 220) + 20; 
lx = 40; ly = 220;
avg_r = integral2(@(x,y)(x .^ 2 + y .^ 2) .^ 0.5, -lx/2, lx/2, -ly/2, ly/2) / (lx * ly); 

% params related to the motion requirements
global angular_acc mu_k rotation_angle_swing update_freq
mu_k = envionment_setting.mu_k; % 0.3; % kinematic friction coefficient
T = motion_req.T_tot_period;
update_freq = motion_req.update_freq;
rotation_angle_swing = motion_req.rotation_angle_swing; % in radius


%% Build the robot's model.
wholebody;

%% Plan a path and generate a trajectory satisfying the given requirements.
[traj_points, Steps, angular_acc_swing] = generate_trajectory;


%% Generate a time profile of joint angles and test on it.
angular_acc_r_swing = angular_acc_swing(1:Steps(2));
angular_acc_l_swing = angular_acc_swing((Steps(2)+1):end);
[the_tot, f_tot] = optimize_joint_angles(traj_points, Steps, angular_acc_r_swing, angular_acc_l_swing);

%% Post-processing
% cut the force
[~, I] = find(f_tot > 23);
f_tot(I) = 23;

% process the joint angles
the_final = [the_tot(:, 1:6), the_tot(:, 8:13)];

% thrusts of the two ducted fans
f_final = zeros(length(f_tot), 2);
partition = 20 + Steps(1) + Steps(2) + Steps(3);
f_final(1:partition, 2) = f_tot(1:partition); % right ducted fan's working
f_final((partition+1):end, 1) = f_tot((partition+1):end); % left ducted fan's working

% combine to make up the final output
output = [the_final, f_final];


end