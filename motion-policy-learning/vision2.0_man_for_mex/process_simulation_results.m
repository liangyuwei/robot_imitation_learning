function process_simulation_results
%% This function displays the simulation results under different configurations.
% examples:
% plot3(position_left_ankle_swing(:, 1), position_left_ankle_swing(:, 2), position_left_ankle_swing(:, 3), 'rx');
% plot3(position_left_ducted_fan_swing(:, 1), position_left_ducted_fan_swing(:, 2), position_left_ducted_fan_swing(:, 3), 'rx');
% plot3(position_main_body_com(:, 1), position_main_body_com(:, 2), position_main_body_com(:, 3), 'rx');

% items:
% 'mu_k', 'angular_acc', 'traj_radius', 'traj_height', 
% 'the_swing', 'f', 
% 'position_left_ankle_swing' 'position_left_ducted_fan_swing', 'position_main_body_com', 'err');

% Items to be displayed: theta, f

%% Display the difference between different mu_k
% simu_0 simu_5 simu_1
load('simulations_results_traj_planning_swing_phase_0.mat'); f0 = f; theta0 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_5.mat'); f1 = f; theta1 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_1.mat'); f2 = f; theta2 = the_swing(:, 6);

% plot theta
figure;
plot(1:length(theta0), theta0, 'r.'); hold on;
plot(1:length(theta1), theta1, 'bx');
plot(1:length(theta2), theta2, 'go');
grid on;
legend('mu_k = 0.3', 'mu_k = 0.5', 'mu_k = 0.8');
xlabel('Steps'); ylabel('Rotation angle of the left ankle / deg');
title('angular acc = 1 deg/s^2, traj radius = 380, traj height = 300');
axis([0, 50, -90, 0]);

% plot f
figure;
plot(1:length(f0), f0, 'r.'); hold on;
plot(1:length(f1), f1, 'bx');
plot(1:length(f2), f2, 'go');
grid on;
legend('mu_k = 0.3', 'mu_k = 0.5', 'mu_k = 0.8');
xlabel('Steps'); ylabel('Left ducted fan''s thrust / N');
title('angular acc = 1 deg/s^2, traj radius = 380, traj height = 300');

%% different angular_acc 
%simu_0 simu_6 simu_2
load('simulations_results_traj_planning_swing_phase_0.mat'); f0 = f; theta0 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_6.mat'); f1 = f; theta1 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_2.mat'); f2 = f; theta2 = the_swing(:, 6);

% plot theta
figure;
plot(1:length(theta0), theta0, 'r.'); hold on;
plot(1:length(theta1), theta1, 'bx');
plot(1:length(theta2), theta2, 'go');
grid on;
legend('angular acc = 1 deg/s^2', 'angular acc = 3 deg/s^2', 'angular acc = 5 deg/s^2');
xlabel('Steps'); ylabel('Rotation angle of the left ankle / deg');
title('mu_k = 0.3, traj radius = 380, traj height = 300');
axis([0, 50, -90, 0]);

% plot f
figure;
plot(1:length(f0), f0, 'r.'); hold on;
plot(1:length(f1), f1, 'bx');
plot(1:length(f2), f2, 'go');
grid on;
legend('angular acc = 1 deg/s^2', 'angular acc = 3 deg/s^2', 'angular acc = 5 deg/s^2');
xlabel('Steps'); ylabel('Left ducted fan''s thrust / N');
title('mu_k = 0.3, traj radius = 380, traj height = 300');

%% different traj radius
%simu_0 simu_3 simu_7
load('simulations_results_traj_planning_swing_phase_0.mat'); f0 = f; theta0 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_3.mat'); f1 = f; theta1 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_7.mat'); f2 = f; theta2 = the_swing(:, 6);

% plot theta
figure;
plot(1:length(theta0), theta0, 'r.'); hold on;
plot(1:length(theta1), theta1, 'bx');
plot(1:length(theta2), theta2, 'go');
grid on;
legend('traj radius = 380', 'traj radius = 480', 'traj radius = 580');
xlabel('Steps'); ylabel('Rotation angle of the left ankle / deg');
title('mu_k = 0.3, angular acc = 1 deg/s^2, traj height = 300');
axis([0, 50, -90, 0]);

% plot f
figure;
plot(1:length(f0), f0, 'r.'); hold on;
plot(1:length(f1), f1, 'bx');
plot(1:length(f2), f2, 'go');
grid on;
legend('traj radius = 380', 'traj radius = 480', 'traj radius = 580');
xlabel('Steps'); ylabel('Left ducted fan''s thrust / N');
title('mu_k = 0.3, angular acc = 1 deg/s^2, traj height = 300');


%% different traj height
%simu_0 simu_4 simu_8
load('simulations_results_traj_planning_swing_phase_0.mat'); f0 = f; theta0 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_4.mat'); f1 = f; theta1 = the_swing(:, 6);
load('simulations_results_traj_planning_swing_phase_8.mat'); f2 = f; theta2 = the_swing(:, 6);

% plot theta
figure;
plot(1:length(theta0), theta0, 'r.'); hold on;
plot(1:length(theta1), theta1, 'bx');
plot(1:length(theta2), theta2, 'go');
grid on;
legend('traj height = 300', 'traj height = 350', 'traj height = 400');
xlabel('Steps'); ylabel('Rotation angle of the left ankle / deg');
title('mu_k = 0.3, angular acc = 1 deg/s^2, traj radius = 380');
axis([0, 50, -90, 0]);

% plot f
figure;
plot(1:length(f0), f0, 'r.'); hold on;
plot(1:length(f1), f1, 'bx');
plot(1:length(f2), f2, 'go');
grid on;
legend('traj height = 300', 'traj height = 350', 'traj height = 400');
xlabel('Steps'); ylabel('Left ducted fan''s thrust / N');
title('mu_k = 0.3, angular acc = 1 deg/s^2, traj radius = 380');


end