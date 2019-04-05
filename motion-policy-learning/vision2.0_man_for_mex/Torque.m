function T = Torque(link,pos_target_joint,F,F_position)
%% This function ...
% link - correlated, rotating joints;
% pos_target_joint - the position of the target joint(whose torque is computed within this function) w.r.t the base
%                  - also, the 
% F - the force required for the ducted fan set on the same side of this joint to generate
% F_position - the position of the ducted fan w.r.t the base

%% Prepare
global uLINK

% find joint's index
NO = find(link);

%% Calculate the joint torque
% calculation of gravitational moment
M = 0;
L = 0;
for i = 1 : length(NO)
    % total mass of the related joints(links)
    M = M + uLINK(link(i)).m; 
    % gravitational moment
    L = L + uLINK(link(i)).m * abs(uLINK(link(i)).c(2)-pos_target_joint(2)); 
end
L_G = L * 9.8 * 0.001;

% torque provided by the ducted fan
L_F = F * abs(F_position(2) - pos_target_joint(2)) * 0.001; % the horizontal distance between target joint and the ducted fan

% joint torque
T = L_F - L_G; % 0.001 - mm --> m


end
