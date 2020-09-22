function plot_local_frames(xyz, rot)
%% Plot local frame by origin of position and rotation matrix.

length = 0.1;
plot3(xyz(1), xyz(2), xyz(3), 'bo'); hold on; grid on;
quiver3(xyz(1), xyz(2), xyz(3), rot(1, 1) * length, rot(2, 1) * length, rot(3, 1) * length, 'r', 'LineWidth', 0.5, 'MaxHeadSize', 0.5); % x-axis
quiver3(xyz(1), xyz(2), xyz(3), rot(1, 2) * length, rot(2, 2) * length, rot(3, 2) * length, 'g', 'LineWidth', 0.5, 'MaxHeadSize', 0.5); % y-axis
quiver3(xyz(1), xyz(2), xyz(3), rot(1, 3) * length, rot(2, 3) * length, rot(3, 3) * length, 'b', 'LineWidth', 0.5, 'MaxHeadSize', 0.5); % z-axis

end