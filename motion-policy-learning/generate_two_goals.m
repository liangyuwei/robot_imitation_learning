function [goal_l, goal_r] = generate_two_goals(x_contact_origin, l_euler_final)


l_x_final = x_contact_origin' + eul2rotm([l_euler_final(3), l_euler_final(2), l_euler_final(1)]) * [-0.01, 0, 0]'; %[-0.12, 0, 0]';
goal_l = [l_x_final', l_euler_final];

r_euler_final = l_euler_final; r_euler_final(1:2) = -r_euler_final(1:2); r_euler_final(3) = pi + r_euler_final(3);
r_x_final = x_contact_origin' + eul2rotm([r_euler_final(3), r_euler_final(2), r_euler_final(1)]) * [-0.01, 0, 0]'; %[-0.19, 0, 0]';
goal_r = [r_x_final', r_euler_final];

end