function plot_joints_comp_subplots(joint_trajs_1, joint_trajs_2, title_name, legend_1, legend_2)
%% This function plots joint trajectories for arm joints in a subplot.
% Input trajectories should be of size DOF x N.


%% Prep
[DOF, num_datapoints] = size(joint_trajs_1);


%%
figure;
for d = 1 : DOF
    subplot(DOF, 1, d);
    p1 = plot(1:num_datapoints, joint_trajs_1(d, :), 'b--'); hold on; grid on;
    p2 = plot(1:0.5:num_datapoints, joint_trajs_2(d, :), 'r--'); % the step should be inconsistent with outside setting
    xlabel('t', 'FontSize', 16); ylabel(['J', num2str(d)], 'FontSize', 16);
    legend([p1(1), p2(1)], legend_1, legend_2);%, 'Location', 'NorthEastOutside', 'FontSize', 16); % Location might not be available in 2016b
end
% for MATLAB 2019b
% sgtitle(title_name, 'FontSize', 16); % title for a grid of subplots
% for MATLAB 2016b
suptitle(title_name); % title for a grid of subplots


end