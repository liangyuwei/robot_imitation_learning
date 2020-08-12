function plot_3d_subplots(traj_1, traj_2, title_name, legend_1, legend_2)
%% This function plots the comparison of two trajectories, with each dimension plotted in one subplot.
% traj_1 and traj_2 should be of the same size, and the size should be 3 x N 


%% 
num_datapoints = size(traj_1, 2);
assert(num_datapoints == size(traj_2, 2), 'The input trajectories should be of the same size!!!');

%%
figure;
subplot(3, 1, 1); 
p1 = plot(1:num_datapoints, traj_1(1, :), 'r--'); hold on; grid on;
p2 = plot(1:num_datapoints, traj_2(1, :), 'g--');
xlabel('Path point', 'FontSize', 16); ylabel('X', 'FontSize', 16);
legend([p1(1), p2(1)], legend_1, legend_2, 'Location', 'NorthEastOutside', 'FontSize', 16);

subplot(3, 1, 2);
p1 = plot(1:num_datapoints, traj_1(2, :), 'r--'); hold on; grid on;
p2 = plot(1:num_datapoints, traj_2(2, :), 'g--');
xlabel('Path point', 'FontSize', 16); ylabel('Y', 'FontSize', 16);
legend([p1(1), p2(1)], legend_1, legend_2, 'Location', 'NorthEastOutside', 'FontSize', 16);

subplot(3, 1, 3);
p1 = plot(1:num_datapoints, traj_1(3, :), 'r--'); hold on; grid on;
p2 = plot(1:num_datapoints, traj_2(3, :), 'g--');
xlabel('Path point', 'FontSize', 16); ylabel('Z', 'FontSize', 16);
legend([p1(1), p2(1)], legend_1, legend_2, 'Location', 'NorthEastOutside', 'FontSize', 16);

sgtitle(title_name, 'FontSize', 16); % title for a grid of subplots






end