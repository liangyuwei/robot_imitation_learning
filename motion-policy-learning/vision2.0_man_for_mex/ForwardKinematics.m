function ForwardKinematics(j)
%% This function calculates the Absolute Transform.

global uLINK;

%% Stop at the last joint whose child is 0
if j == 0
    return;
end

%% For non-end joint
if j ~= 1
    %% using mother's position and pose
    mom = uLINK(j).mother;
    
    %% calculate absolute transform iteratively
    uLINK(j).p = uLINK(mom).R * uLINK(j).b + uLINK(mom).p;
    % post-multiply, relative transformation;
    % Rodrigues() is the transform matrix between two adjacent joints
    uLINK(j).R = uLINK(mom).R * Rodrigues(uLINK(j).a, uLINK(j).q);
    
    %% the absolute position of the Center of Mass(w.r.t the world frame)
    uLINK(j).c = uLINK(j).R * uLINK(j).c + uLINK(j).p;

end

% Iteratively calculate the transform matrix for each joint
ForwardKinematics(uLINK(j).sister);
ForwardKinematics(uLINK(j).child);

end


