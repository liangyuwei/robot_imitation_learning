function update_robot_joint
%% This function updates the robot's joint angl, for later use of forward kinematics

global uLINK th
for i = 1:6
    uLINK(i+1).q = th(i);
end

end