function ForwardKinematics(j)
%% This function computes the rotation matrix and displacement for each link.
% Usage: Start with ForwardKinematics(1), i.e. the base link.

%% Define global variable for storing the results
global uLINK

%% Start recursive calculation
if j == 0 % nothing
    return;
end
if j ~= 1 % if not the root link, then it has a parent link
    mom = uLINK(j).mom; % get its parent link's index
    uLINK(j).p = uLINK(mom).R * uLINK(j).b + uLINK(mom).p; % compute current link's p using its parent's R and p
    uLINK(j).R = uLINK(mom).R * Rodrigues(uLINK(j).a, uLINK(j).q); % q is the angle of rotation
end

ForwardKinematics(uLINK(j).child); % compute its child's R and p


end