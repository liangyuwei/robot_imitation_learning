function display_frame_change(traj_l, traj_r)
%% This function displays the changing process of dual-arms' end-effectors' frames.
% traj_{l/r} is of size (6, len_samples)

% setup
len_samples = size(traj_l, 2);

% 
figure;
scale = 0.2; % scale of the length of arrow
id = round(linspace(1, len_samples, 100));
for n = 1 : length(id)
    l_pos = traj_l(1:3, id(n)); % position of one point
    l_eul = traj_l(4:6, id(n)); % euler angle of one point, 'xyz' for RViz
    r_pos = traj_r(1:3, id(n));
    r_eul = traj_r(4:6, id(n));
    l_rotm = eul2rotm([l_eul(3), l_eul(2), l_eul(1)]); % euler angle 'zyx' for matlab
    r_rotm = eul2rotm([r_eul(3), r_eul(2), r_eul(1)]); % euler angle 'zyx' for matlab
    l_u = l_rotm(:, 1); l_v = l_rotm(:, 2); l_w = l_rotm(:, 3);
    r_u = r_rotm(:, 1); r_v = r_rotm(:, 2); r_w = r_rotm(:, 3);

    quiver3(l_pos(1), l_pos(2), l_pos(3), l_u(1), l_u(2), l_u(3), scale, 'Color', 'r'); % x axis
    hold on; grid on;
    quiver3(l_pos(1), l_pos(2), l_pos(3), l_v(1), l_v(2), l_v(3), scale, 'Color', 'g'); % y axis
    quiver3(l_pos(1), l_pos(2), l_pos(3), l_w(1), l_w(2), l_w(3), scale, 'Color', 'b'); % z axis
    quiver3(r_pos(1), r_pos(2), r_pos(3), r_u(1), r_u(2), r_u(3), scale, 'Color', 'r'); % x axis
    quiver3(r_pos(1), r_pos(2), r_pos(3), r_v(1), r_v(2), r_v(3), scale, 'Color', 'g'); % y axis
    quiver3(r_pos(1), r_pos(2), r_pos(3), r_w(1), r_w(2), r_w(3), scale, 'Color', 'b'); % z axis

    axis([0, 0.6, -0.5, 0.5, 0, 0.6]);
    view(120, 45);
    xlabel('x'); ylabel('y'); zlabel('z');

    hold off;
    pause(0.01);
end


end