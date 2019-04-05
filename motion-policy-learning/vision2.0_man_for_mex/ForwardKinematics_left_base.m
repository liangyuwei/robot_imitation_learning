function ForwardKinematics_left_base
%% This function calculates the Absolute Transform from the left ankle2 to other joints

global uLINK;

%% For non-end joint
% the position and pos of the new base, frame 13, have been set in wholdbody.m 
uLINK(8).c = uLINK(8).R * uLINK(8).c + uLINK(8).p;
last = 8;
for i = 7:-1:1
    if i == 7
        transform_i_to_last = Rodrigues(uLINK(last).a, 0);
    else
        transform_i_to_last = Rodrigues(uLINK(last).a, uLINK(last).q);
    end
    uLINK(i).R = uLINK(last).R * transform_i_to_last';
    uLINK(i).p = uLINK(last).p - uLINK(i).R * uLINK(last).b;   
    uLINK(i).c = uLINK(i).R * uLINK(i).c + uLINK(i).p;
    last = i;
end

% from 10 to 15, using forward calculation
for i = 9:15
    uLINK(i).p = uLINK(last).R * uLINK(i).b + uLINK(last).p;
    uLINK(i).R = uLINK(last).R * Rodrigues(uLINK(i).a, uLINK(i).q);
    uLINK(i).c = uLINK(i).R * uLINK(i).c + uLINK(i).p;
    last = i;
end


end
