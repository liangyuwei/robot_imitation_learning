function [traj_points, Steps, angular_acc_swing] = generate_trajectory
%% This function generates the trajectory of the whole process.
% For our experiments, the whole process is divided into six parts, i.e.
% R_up --> R_swing --> R_down --> L_up --> L_swing --> L_down


%% Prepare parameters
global rotation_angle_swing traj_height traj_radius full_length
global T update_freq left_pivot right_pivot joint_feet_displacement
% T contains the periods of six phases.

%% [1]R_up
% left foot center is the pivot
% plan the right ANKLE2's trajectory
steps_r_up = T(1) * update_freq;
P_ini = [100, 0, joint_feet_displacement];
P_end = left_pivot + [traj_radius, 0, traj_height]; % left swing pivot + joint_feet_displacement + traj settings
P_mid = (P_ini + P_end) / 2 + [10, 0, -50];
P_tmp_matrix = [P_ini; P_mid; P_end];
x0 = P_tmp_matrix(:, 1);
z0 = P_tmp_matrix(:, 3);

% fit a curve
p = polyfit(x0,z0,2);

% get trajectory points
traj_points_r_up = zeros(3, steps_r_up);
for n = 1:steps_r_up
   % get x
   x = P_ini(1) + (n-1) * (P_end(1) - P_ini(1)) / (steps_r_up - 1);
   
   % get z
   z = p(1) * x .^2 + p(2) * x .^1 + p(3);
   
   % record the trajectory points
   traj_points_r_up(:, n) = [x, 0, z]';
   
end
    
% display the results
% figure;
% plot3(traj_points_r_up(1, :), traj_points_r_up(2, :), traj_points_r_up(3, :), 'b.');
% grid on;


%% [2]R_swing
% left foot center is the pivot
% plan the right ANKLE2's trajectory
T_r_swing = T(2);
steps_r_swing = T(2) * update_freq;
traj_points_r_swing = zeros(3, steps_r_swing);
% kinematic parameters, in radius
angular_acc_max = 2 * pi * rotation_angle_swing / T_r_swing ^2; % in radius, note that we add a '-'
angular_acc_z = @(t) angular_acc_max * sin(2*pi/T_r_swing * t); 
% angular_vel_z = @(t) angular_acc_max * T_r_swing / (2*pi) * (1 - cos(2*pi/T_r_swing*t)); 
angle_z = @(t) angular_acc_max * T_r_swing / (2*pi) * (t - T_r_swing/(2*pi)*sin(2*pi/T_r_swing*t));
% display the kinematic parameters
%{
ToDeg = 180/pi;
% angular acceleration
figure;
plot(0:0.1:20, angular_acc_z(0:0.1:20)*ToDeg, 'b-'); grid on;
xlabel('Time/s'); ylabel('angular acceleration/(deg * s^-2)');
title('Planned angular acceleration');
% angular velocity
figure;
plot(0:0.1:20, angular_vel_z(0:0.1:20)*ToDeg, 'r-'); grid on;
xlabel('Time/s'); ylabel('angular velocity/(deg * s^-1)');
title('Planned angular velocity');
% angle displacement
figure;
plot(0:0.1:20, angle_z(0:0.1:20)*ToDeg, 'g-'); grid on;
xlabel('Time/s'); ylabel('angle/deg');
title('Planned angle trajectory');
%}
% swing trajectory of an arc
angular_acc_r_swing = zeros(1, steps_r_swing);
for n = 1:steps_r_swing
    % current time
    t = T_r_swing / (steps_r_swing - 1) * (n-1);
    
    % current kinematic parameters
    dw_z = angular_acc_z(t);
%     w_z = angular_vel_z(t);
    fi_angle = angle_z(t);
    
    % add current trajectory point
    traj_points_r_swing(:, n) = [traj_radius * cos(fi_angle) + left_pivot(1);
                                 traj_radius * sin(fi_angle) + left_pivot(2);
                                 traj_height + left_pivot(3)];
    angular_acc_r_swing(n) = dw_z;
    
end

% display the planned trajectory
% figure;
% plot3(traj_points_r_swing(1, :), traj_points_r_swing(2, :), traj_points_r_swing(3, :), 'b.');
% grid on; hold on;
% plot3(traj_points_r_swing(1, 1), traj_points_r_swing(2, 1), traj_points_r_swing(3, 1), 'go');
% plot3(traj_points_r_swing(1, end), traj_points_r_swing(2, end), traj_points_r_swing(3, end), 'rx');
% xlabel('x'); ylabel('y'); zlabel('z');


%% [3]R_down
% left foot as the pivot
% plan the right ANKLE2's trajectory
steps_r_down = T(3) * update_freq;
P_ini = [left_pivot(1), traj_radius + left_pivot(2), traj_height];
P_end = left_pivot + [0, 20, 0] + [0, full_length, 0] + [0, 0, joint_feet_displacement];
P_turn = P_end + [0, -200, 0] ;
P_mid = (P_ini + P_turn) / 2 + [0, 30, 20];
y0 = [P_ini(2), P_mid(2), P_turn(2)];
z0 = [P_ini(3), P_mid(3), P_turn(3)];
% fit a curve
p = polyfit(y0,z0,2);

% get the trajectory points
traj_points_r_down = zeros(3, steps_r_down);
for n = 1:steps_r_down
    
    y = P_ini(2) + (n - 1) * (P_end(2) - P_ini(2)) / (steps_r_down - 1);
    z = p(1) * y .^2 + p(2) * y .^1 + p(3);
    if y > P_turn(2)
        % when the front leg reach the other side of the ditch, keep
        % shifting to spare room(200 mm) for the left leg to stand 
        z = P_turn(3);
    end

    % record 
    traj_points_r_down(:, n) = [left_pivot(1), y, z]';
    
end

% display the results
% figure;
% plot3(traj_points_r_down(1, :), traj_points_r_down(2, :), traj_points_r_down(3, :), 'b.');
% grid on; hold on;
% plot3(traj_points_r_down(1, 1), traj_points_r_down(2, 1), traj_points_r_down(3, 1), 'go');
% plot3(traj_points_r_down(1, end), traj_points_r_down(2, end), traj_points_r_down(3, end), 'rx');


%% [4]L_up
% right foot as the pivot
% plan the left ANKLE2's trajectory
steps_l_up = T(4)*update_freq;
traj_points_l_up = zeros(3, steps_l_up);
% get the initial position and final position
P_ini = (left_pivot + [0, 20, 0]) + [0, 0, joint_feet_displacement];
P_turn = P_ini + [0, 200, 0] ; % shuffle
P_end = left_pivot + [0, 20, 0] + [0, full_length, 0] + [0, - traj_radius, traj_height]; % transition end point
P_mid = (P_turn + P_end) / 2 + [0, -20, 30];
y0 = [P_turn(2), P_mid(2), P_end(2)];
z0 = [P_turn(3), P_mid(3), P_end(3)];
p = polyfit(y0,z0,2);

% lift-up phase
for n = 1:steps_l_up

    y = P_ini(2) + (n - 1) * (P_end(2) - P_ini(2)) / (steps_l_up - 1);
    z = p(1) * y .^2 + p(2) * y .^1 + p(3);
    if y <= P_turn(2)
        z = P_turn(3);
    end
    % record
    traj_points_l_up(:, n) = [P_ini(1), y, z]';
end

% display the results
% figure;
% plot3(traj_points_l_up(1, :), traj_points_l_up(2, :), traj_points_l_up(3, :), 'b.');
% grid on; hold on;
% plot3(traj_points_l_up(1, 1), traj_points_l_up(2, 1), traj_points_l_up(3, 1), 'go');
% plot3(traj_points_l_up(1, end), traj_points_l_up(2, end), traj_points_l_up(3, end), 'rx');


%% [5]L_swing
% right foot center is the pivot
% plan the left ANKLE2's trajectory
T_l_swing = T(5);
steps_l_swing = T(5) * update_freq;
traj_points_l_swing = zeros(3, steps_l_swing);
% kinematic parameters, in radius
angular_acc_max = 2 * pi * rotation_angle_swing / T_l_swing ^2; % in radius, note that we add a '-'
angular_acc_z = @(t) angular_acc_max * sin(2*pi/T_l_swing * t); 
% angular_vel_z = @(t) angular_acc_max * T_r_swing / (2*pi) * (1 - cos(2*pi/T_r_swing*t)); 
angle_z = @(t) angular_acc_max * T_l_swing / (2*pi) * (t - T_l_swing/(2*pi)*sin(2*pi/T_l_swing*t));
% display the kinematic parameters
%{
ToDeg = 180/pi;
% angular acceleration
figure;
plot(0:0.1:20, angular_acc_z(0:0.1:20)*ToDeg, 'b-'); grid on;
xlabel('Time/s'); ylabel('angular acceleration/(deg * s^-2)');
title('Planned angular acceleration');
% angular velocity
figure;
plot(0:0.1:20, angular_vel_z(0:0.1:20)*ToDeg, 'r-'); grid on;
xlabel('Time/s'); ylabel('angular velocity/(deg * s^-1)');
title('Planned angular velocity');
% angle displacement
figure;
plot(0:0.1:20, angle_z(0:0.1:20)*ToDeg, 'g-'); grid on;
xlabel('Time/s'); ylabel('angle/deg');
title('Planned angle trajectory');
%}
% swing trajectory of an arc
angular_acc_l_swing = zeros(1, steps_l_swing);
for n = 1:steps_l_swing
    % current time
    t = T_l_swing / (steps_l_swing - 1) * (n-1);
    
    % current kinematic parameters
    dw_z = angular_acc_z(t);
%     w_z = angular_vel_z(t);
    fi_angle = angle_z(t);
    
    % add current trajectory point
    traj_points_l_swing(:, n) = [traj_radius * sin(fi_angle) + right_pivot(1);
                               - traj_radius * cos(fi_angle) + right_pivot(2);
                                 traj_height + right_pivot(3)];
    angular_acc_l_swing(n) = dw_z;
    
end

% display the planned trajectory
% figure;
% plot3(traj_points_l_swing(1, :), traj_points_l_swing(2, :), traj_points_l_swing(3, :), 'b.');
% grid on; hold on;
% plot3(traj_points_l_swing(1, 1), traj_points_l_swing(2, 1), traj_points_l_swing(3, 1), 'go');
% plot3(traj_points_l_swing(1, end), traj_points_l_swing(2, end), traj_points_l_swing(3, end), 'rx');
% xlabel('x'); ylabel('y'); zlabel('z');


%% [6]L_down
% right foot as the pivot
% plan the left ANKLE2's trajectory
steps_l_down = T(6) * update_freq;
P_ini = right_pivot + [traj_radius, 0, traj_height];
P_end = [100, right_pivot(2), joint_feet_displacement];
P_mid = (P_ini + P_end) / 2 + [20, 0, -30];
P_tmp_matrix = [P_ini; P_mid; P_end];
x0 = P_tmp_matrix(:, 1);
z0 = P_tmp_matrix(:, 3);

% fit a curve
p = polyfit(x0,z0,2);

% get trajectory points
traj_points_l_down = zeros(3, steps_l_down);
for n = 1:steps_l_down
   % get x
   x = P_ini(1) + (n-1) * (P_end(1) - P_ini(1)) / (steps_l_down - 1);
   
   % get z
   z = p(1) * x .^2 + p(2) * x .^1 + p(3);
   
   % record the trajectory points
   traj_points_l_down(:, n) = [x, right_pivot(2), z]';
   
end
    
% display the results
% figure;
% plot3(traj_points_l_down(1, :), traj_points_l_down(2, :), traj_points_l_down(3, :), 'b.');
% grid on; hold on;
% plot3(traj_points_l_down(1, 1), traj_points_l_down(2, 1), traj_points_l_down(3, 1), 'go');
% plot3(traj_points_l_down(1, end), traj_points_l_down(2, end), traj_points_l_down(3, end), 'rx');
% xlabel('x'); ylabel('y'); zlabel('z');


%% Display the trajectories as a whole
%{
figure;
plot3(traj_points_r_up(1, :), traj_points_r_up(2, :), traj_points_r_up(3, :), 'b.');
grid on; hold on;
plot3(traj_points_r_up(1, 1), traj_points_r_up(2, 1), traj_points_r_up(3, 1), 'go');
plot3(traj_points_r_up(1, end), traj_points_r_up(2, end), traj_points_r_up(3, end), 'rx');

plot3(traj_points_r_swing(1, :), traj_points_r_swing(2, :), traj_points_r_swing(3, :), 'b.');
plot3(traj_points_r_swing(1, 1), traj_points_r_swing(2, 1), traj_points_r_swing(3, 1), 'go');
plot3(traj_points_r_swing(1, end), traj_points_r_swing(2, end), traj_points_r_swing(3, end), 'rx');

plot3(traj_points_r_down(1, :), traj_points_r_down(2, :), traj_points_r_down(3, :), 'b.');
plot3(traj_points_r_down(1, 1), traj_points_r_down(2, 1), traj_points_r_down(3, 1), 'go');
plot3(traj_points_r_down(1, end), traj_points_r_down(2, end), traj_points_r_down(3, end), 'rx');

plot3(traj_points_l_up(1, :), traj_points_l_up(2, :), traj_points_l_up(3, :), 'b.');
plot3(traj_points_l_up(1, 1), traj_points_l_up(2, 1), traj_points_l_up(3, 1), 'go');
plot3(traj_points_l_up(1, end), traj_points_l_up(2, end), traj_points_l_up(3, end), 'rx');

plot3(traj_points_l_swing(1, :), traj_points_l_swing(2, :), traj_points_l_swing(3, :), 'b.');
plot3(traj_points_l_swing(1, 1), traj_points_l_swing(2, 1), traj_points_l_swing(3, 1), 'go');
plot3(traj_points_l_swing(1, end), traj_points_l_swing(2, end), traj_points_l_swing(3, end), 'rx');

plot3(traj_points_l_down(1, :), traj_points_l_down(2, :), traj_points_l_down(3, :), 'b.');
plot3(traj_points_l_down(1, 1), traj_points_l_down(2, 1), traj_points_l_down(3, 1), 'go');
plot3(traj_points_l_down(1, end), traj_points_l_down(2, end), traj_points_l_down(3, end), 'rx');

xlabel('x'); ylabel('y'); zlabel('z');
axis([-200, 800, 0, 1000]);
%}

%% Return other parameters
Steps = [steps_r_up, steps_r_swing, steps_r_down, steps_l_up, steps_l_swing, steps_l_down];
traj_points = [traj_points_r_up, traj_points_r_swing, traj_points_r_down, ...
               traj_points_l_up, traj_points_l_swing, traj_points_l_down];
angular_acc_swing = [angular_acc_r_swing, angular_acc_l_swing];

end