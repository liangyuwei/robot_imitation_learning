function ForwardKinematics_right_base
%% This function calculates the Absolute Transform from the right ankle2 to other joints

global uLINK;

%% For non-end joint
% the position and pos of the new base, frame 15, have been set in wholdbody.m 
uLINK(15).c = uLINK(15).R * uLINK(15).c + uLINK(15).p;
last = 15;
for i = 14:-1:9
    if i == 14
        transform_i_to_last = Rodrigues(uLINK(last).a, 0);
    else
        transform_i_to_last = Rodrigues(uLINK(last).a, uLINK(last).q);
    end
    uLINK(i).R = uLINK(last).R * transform_i_to_last';
    uLINK(i).p = uLINK(last).p - uLINK(i).R * uLINK(last).b;   
    uLINK(i).c = uLINK(i).R * uLINK(i).c + uLINK(i).p;
    last = i;
end

% last = 9, now it should be 1... using the same inverse calculation as above 
transform_i_to_last = Rodrigues(uLINK(last).a, uLINK(last).q);
uLINK(1).R = uLINK(last).R * transform_i_to_last';
uLINK(1).p = uLINK(last).p - uLINK(1).R * uLINK(last).b;
uLINK(1).c = uLINK(1).R * uLINK(1).c + uLINK(1).p;
last = 1;

% from 2 to 8, using forward calculation
for i = 2:8
    uLINK(i).p = uLINK(last).R * uLINK(i).b + uLINK(last).p;
    uLINK(i).R = uLINK(last).R * Rodrigues(uLINK(i).a, uLINK(i).q);
    uLINK(i).c = uLINK(i).R * uLINK(i).c + uLINK(i).p;
    last = i;
end

end
