function display_frame_change(traj_l, traj_r, record, gif_name)
%% This function displays the changing process of dual-arms' end-effectors' frames.
% traj_{l/r} is of size (6, len_samples)

% setup
len_samples = size(traj_l, 2);

% 
figure;
scale = 0.1; % scale of the length of arrow
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

    % draw the whole trajectory
    plot3(traj_l(1, 1), traj_l(2, 1), traj_l(3, 1), 'go'); hold on; grid on;
    plot3(traj_l(1, 2:end), traj_l(2, 2:end), traj_l(3, 2:end), 'b-');
    plot3(traj_l(1, end), traj_l(2, end), traj_l(3, end), 'ro'); 
    plot3(traj_r(1, 1), traj_r(2, 1), traj_r(3, 1), 'go'); 
    plot3(traj_r(1, 2:end), traj_r(2, 2:end), traj_r(3, 2:end), 'b-');
    plot3(traj_r(1, end), traj_r(2, end), traj_r(3, end), 'ro'); 
    
    % draw the moving frame
    quiver3(l_pos(1), l_pos(2), l_pos(3), l_u(1), l_u(2), l_u(3), scale, 'Color', 'r'); % x axis
%     hold on; grid on;
    quiver3(l_pos(1), l_pos(2), l_pos(3), l_v(1), l_v(2), l_v(3), scale, 'Color', 'g'); % y axis
    quiver3(l_pos(1), l_pos(2), l_pos(3), l_w(1), l_w(2), l_w(3), scale, 'Color', 'b'); % z axis
    quiver3(r_pos(1), r_pos(2), r_pos(3), r_u(1), r_u(2), r_u(3), scale, 'Color', 'r'); % x axis
    quiver3(r_pos(1), r_pos(2), r_pos(3), r_v(1), r_v(2), r_v(3), scale, 'Color', 'g'); % y axis
    quiver3(r_pos(1), r_pos(2), r_pos(3), r_w(1), r_w(2), r_w(3), scale, 'Color', 'b'); % z axis

    axis([0.0, 0.8, -0.4, 0.4, 0.2, 0.6]);%([0, 0.6, -0.5, 0.5, 0, 0.6]);
    view(100, 45);%(120, 45);
    xlabel('x'); ylabel('y'); zlabel('z');

    hold off;
    
    % store the frames for making gif later
    if record
        f = getframe;
        im = frame2im(f);
        [I, map] = rgb2ind(im, 256);
        % set Loopcount as Inf for the animation to play forever
        if n==1
            imwrite(I, map, gif_name, 'gif', 'Loopcount', Inf, 'DelayTime', 0.01);
        else
            imwrite(I, map, gif_name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.01);
        end
    end
    
    % pause
    if n == 1
        pause; % pause for video recording...
    end
    pause(0.01);

end


end