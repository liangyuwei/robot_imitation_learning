function test_on_crossing_obstacles_with_swing
%% This function generates needed variables to enable the 'crossing_obstacles_with_swing.m'.


%% Needed parameters
traj_setting.end_pos_right_ankle = [0, 920, 0];
traj_setting.swing_height = 300;
traj_setting.swing_radius = 380;

motion_req.T_tot_period = [5, 5, 5, 5, 5, 5];
motion_req.update_freq = 10; % Hertz

envionment_setting.mu_k = 0.3;

motion_req.rotation_angle_swing = pi/2;

%% Call the function to get the results
swing_radius = traj_setting.swing_radius;
swing_height = traj_setting.swing_height;
mu_kinematic = envionment_setting.mu_k;
time_spent_r_swing = motion_req.T_tot_period(2);
time_spent_l_swing = motion_req.T_tot_period(5);


output = crossing_obstacles_with_swing(swing_radius, swing_height, mu_kinematic, time_spent_r_swing, time_spent_l_swing);

%% Display the results
% figure;
% bar(f_tot);


end