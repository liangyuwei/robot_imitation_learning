function fig = plot_3d_subplots_4traj(traj_1, traj_2, traj_3, traj_4, title_name, legend_1, legend_2, legend_3, legend_4, use_legend)
%% This function plots the comparison of two trajectories, with each dimension plotted in one subplot.
% traj_1 and traj_2 should be of the same size, and the size should be 3 x N 


%% Check the input
num_datapoints = size(traj_1, 2);
assert(num_datapoints == size(traj_2, 2), 'The input trajectories should be of the same size!!!');
assert(num_datapoints == size(traj_3, 2), 'The input trajectories should be of the same size!!!');
assert(num_datapoints == size(traj_4, 2), 'The input trajectories should be of the same size!!!');


%% Setups about marker size, line width and font size
mk_size = 24; %20;
mk_indices = 3:5:num_datapoints; %1:3:num_datapoints;

ln_width_1 = 6; %3; % line width for human trajs
ln_width_2 = 4; %2; % line width for retargeted trajs

ld_ft_size = 30; %22; % legend font size
tl_ft_size = 48; %32; %30; % title font size
tk_ft_size = 42; %20; % ticks label font size (a little bit smaller than legend)

text_margin = 0.01; % X, Y, Z label offset from the boundaries of subplots

tl_offset = 0.08; %0.02; % offset of common y label toward the left boundary (to cope with intersection of the common y label and the ticks label)

ld_len_scale = 3.55; %1.5; % the lengths in the legend


%%
fig = figure;

% set(fig, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);

% t = tiledlayout(3, 1, 'TileSpacing', 'none', 'Padding', 'none');

% nexttile;
h1 = subplot(3, 1, 1); 
p1 = plot(linspace(0, 1, num_datapoints), traj_1(1, :), 'r-', 'LineWidth', ln_width_1); hold on; grid on;
p2 = plot(linspace(0, 1, num_datapoints), traj_2(1, :), 'b--*', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
p3 = plot(linspace(0, 1, num_datapoints), traj_3(1, :), 'g--o', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
% p4 = plot(1:num_datapoints, traj_4(1, :), 'k--^', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
set(h1, 'FontSize', tk_ft_size);
% legend([p1(1), p1(1), p2(1), p3(1), p4(1)], legend_1, 'X', legend_2, legend_3, legend_4, 'FontSize', ld_ft_size); % 'Location', 'NorthEastOutside',
if (use_legend)
    leg = legend([p1(1), p2(1), p3(1)], legend_1, legend_2, legend_3, 'FontSize', ld_ft_size); % 'Location', 'NorthEastOutside',
    leg.ItemTokenSize = leg.ItemTokenSize * ld_len_scale; % increase the length of lines in legend box
end
% way 1
xlim = get(h1, 'xlim'); ylim = get(h1, 'ylim'); 
NW = [min(xlim) max(ylim)] + [diff(xlim) -diff(ylim)] .* [text_margin, 2 * text_margin]; % experimenting to get a proper scaling term
text(NW(1), NW(2), 'X', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontSize', ld_ft_size);
% way 2
% legend(p1(1), 'X', 'Location', 'NorthWest');
% way 3 
% dim = [0.05, 1, 0.3, 0.3]; % normalized units
% annotation('textbox', dim, 'String', 'X', 'FitBoxToText', 'on');
% way 4
% ylabel('X', 'FontSize', ld_ft_size);
% title(title_name, 'FontSize', tl_ft_size);

% nexttile;
h2 = subplot(3, 1, 2);
p1 = plot(linspace(0, 1, num_datapoints), traj_1(2, :), 'r-', 'LineWidth', ln_width_1); hold on; grid on;
p2 = plot(linspace(0, 1, num_datapoints), traj_2(2, :), 'b--*', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
p3 = plot(linspace(0, 1, num_datapoints), traj_3(2, :), 'g--o', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
% p4 = plot(1:num_datapoints, traj_4(2, :), 'k--^', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
set(h2, 'FontSize', tk_ft_size);
xlim = get(h2, 'xlim'); ylim = get(h2, 'ylim'); 
NW = [min(xlim) max(ylim)] + [diff(xlim) -diff(ylim)] .* [text_margin, 2 * text_margin]; % experimenting to get a proper scaling term
text(NW(1), NW(2), 'Y', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontSize', ld_ft_size);


% nexttile;
h3 = subplot(3, 1, 3);
p1 = plot(linspace(0, 1, num_datapoints), traj_1(3, :), 'r-', 'LineWidth', ln_width_1); hold on; grid on;
p2 = plot(linspace(0, 1, num_datapoints), traj_2(3, :), 'b--*', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
p3 = plot(linspace(0, 1, num_datapoints), traj_3(3, :), 'g--o', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
% p4 = plot(1:num_datapoints, traj_4(3, :), 'k--^', 'LineWidth', ln_width_2, 'MarkerSize', mk_size, 'MarkerIndices', mk_indices);
set(h3, 'FontSize', tk_ft_size);
% legend([p1(1), p2(1), p3(1), p4(1)], legend_1, legend_2, legend_3, legend_4, 'Location', 'NorthEastOutside', 'FontSize', 16);
xlim = get(h3, 'xlim'); ylim = get(h3, 'ylim'); 
NW = [min(xlim) max(ylim)] + [diff(xlim) -diff(ylim)] .* [text_margin, 2 * text_margin]; % experimenting to get a proper scaling term
text(NW(1), NW(2), 'Z', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontSize', ld_ft_size);


% traditional title
% sgtitle(title_name, 'FontSize', tl_ft_size); % title for a grid of subplots (this is actually not center aligned, not sure why, but it definitely does not align with title())    
% suptitle(title_name);%, 'FontSize', 16); % title for a grid of subplots


% set title_name to common y label for all subplots
%
pos1 = get(h1, 'position');
pos2 = get(h2, 'position');
pos3 = get(h3, 'position');
height = pos1(2) + pos1(4) - pos3(2); % bottom 1 + height 1 - bottom 3
htmp = axes('position', [pos3(1)-tl_offset, pos3(2), pos1(3), height], 'visible', 'off');
h_label = ylabel(title_name, 'visible', 'on', 'FontSize', tl_ft_size);
%}


% expand the figure window to full screen and set margins between subplots
%{
set(fig, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
% set margins between subplots to zero (must expand the figure window to full screen before proceeding)
subplot_margin = 0.06;
% set(h1, 'XTickLabel', []);
% set(h2, 'XTickLabel', []); % disable xlabels of the above two subplots
pos1 = get(h1, 'position');
pos2 = get(h2, 'position');
pos3 = get(h3, 'position'); % get subplots' current size
total_height = pos1(2) - pos3(2) + pos1(4); % total height
subplot_height = (total_height - 2 * subplot_margin) / 3.0;
pos3(4) = subplot_height;                       % height 3
pos2(2) = pos3(2) + subplot_height + subplot_margin; 
pos2(4) = subplot_height;                       % bottom 2 and height 2
pos1(2) = pos2(2) + subplot_height + subplot_margin; 
pos1(4) = subplot_height;                       % bottom 1 and height 1
set(h1, 'position', pos1);
set(h2, 'position', pos2);
set(h3, 'position', pos3); % reset all subplots' dimension information
%}


end