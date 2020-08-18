function plot_joints_subplots(joint_trajs, title_name)
%% This function plots joint trajectories for arm joints in a subplot.
% Input trajectories should be of size DOF x N.


%% Prep
[DOF, num_datapoints] = size(joint_trajs);


%%
figure;
for d = 1 : DOF
    subplot(DOF, 1, d);
    plot(1:num_datapoints, joint_trajs(d, :), 'b-'); hold on; grid on;
    xlabel('t', 'FontSize', 16); ylabel(['J', num2str(d)], 'FontSize', 16);
end
% for MATLAB 2019b
sgtitle(title_name, 'FontSize', 16); % title for a grid of subplots
% for MATLAB 2016b
% suptitle(title_name); % title for a grid of subplots


end