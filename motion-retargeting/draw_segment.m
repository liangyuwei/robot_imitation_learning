function draw_segment(quat, pos)
%% quat is [w,x,y,z], pos is [x,y,z]

scale = 0.1;

%% Draw origin(joint origin)
plot3(pos(1), pos(2), pos(3), 'MarkerSize', 10, 'MarkerFaceColor', 'r');


%% Draw coordinate frame
rotm = quat2rotm(quat);
shift = rotm;
quiver3(pos(1), pos(2), pos(3), shift(1, 1), shift(2, 1), shift(3, 1), scale, 'Color', 'r'); % x-axis
quiver3(pos(1), pos(2), pos(3), shift(1, 2), shift(2, 2), shift(3, 2), scale, 'Color', 'g'); % y-axis
quiver3(pos(1), pos(2), pos(3), shift(1, 3), shift(2, 3), shift(3, 3), scale, 'Color', 'b'); % z-axis





end