function update_robot_joint(left_or_right_arm)
%% This function updates the robot's joint angl, for later use of forward kinematics

global uLINK th

if(left_or_right_arm)
    offset = [0, -pi/4, pi/2, 0, pi/2, 0];
else
    offset =  [pi, -0.75*pi, -pi/2, -pi, -pi/2, 0];
end

for i = 1:6
    uLINK(i+1).q = th(i) + offset(i);
end


end