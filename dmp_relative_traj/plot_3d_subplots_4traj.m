function plot_3d_subplots_4traj(traj_1, traj_2, traj_3, traj_4, title_name, legend_1, legend_2, legend_3, legend_4)
%% This function plots the comparison of two trajectories, with each dimension plotted in one subplot.
% traj_1 and traj_2 should be of the same size, and the size should be 3 x N 


%% 
num_datapoints = size(traj_1, 2);
assert(num_datapoints == size(traj_2, 2), 'The input trajectories should be of the same size!!!');
assert(num_datapoints == size(traj_3, 2), 'The input trajectories should be of the same size!!!');
assert(num_datapoints == size(traj_4, 2), 'The input trajectories should be of the same size!!!');


%%
figure;
subplot(3, 1, 1); 
p1 = plot(1:num_datapoints, traj_1(1, :), 'r-', 'LineWidth', 2); hold on; grid on;
p2 = plot(1:num_datapoints, traj_2(1, :), 'b--*', 'LineWidth', 1);
p3 = plot(1:num_datapoints, traj_3(1, :), 'g--o', 'LineWidth', 1);
p4 = plot(1:num_datapoints, traj_4(1, :), 'k--^', 'LineWidth', 1);
xlabel('Path point', 'FontSize', 16); ylabel('X', 'FontSize', 16);
legend([p1(1), p2(1), p3(1), p4(1)], legend_1, legend_2, legend_3, legend_4, 'Location', 'NorthEastOutside', 'FontSize', 16);

subplot(3, 1, 2);
p1 = plot(1:num_datapoints, traj_1(2, :), 'r-', 'LineWidth', 2); hold on; grid on;
p2 = plot(1:num_datapoints, traj_2(2, :), 'b--*', 'LineWidth', 1);
p3 = plot(1:num_datapoints, traj_3(2, :), 'g--o', 'LineWidth', 1);
p4 = plot(1:num_datapoints, traj_4(2, :), 'k--^', 'LineWidth', 1);
xlabel('Path point', 'FontSize', 16); ylabel('Y', 'FontSize', 16);
legend([p1(1), p2(1), p3(1), p4(1)], legend_1, legend_2, legend_3, legend_4, 'Location', 'NorthEastOutside', 'FontSize', 16);

subplot(3, 1, 3);
p1 = plot(1:num_datapoints, traj_1(3, :), 'r-', 'LineWidth', 2); hold on; grid on;
p2 = plot(1:num_datapoints, traj_2(3, :), 'b--*', 'LineWidth', 1);
p3 = plot(1:num_datapoints, traj_3(3, :), 'g--o', 'LineWidth', 1);
p4 = plot(1:num_datapoints, traj_4(3, :), 'k--^', 'LineWidth', 1);
xlabel('Path point', 'FontSize', 16); ylabel('Z', 'FontSize', 16);
legend([p1(1), p2(1), p3(1), p4(1)], legend_1, legend_2, legend_3, legend_4, 'Location', 'NorthEastOutside', 'FontSize', 16);

sgtitle(title_name, 'FontSize', 16); % title for a grid of subplots
% suptitle(title_name);%, 'FontSize', 16); % title for a grid of subplots



end