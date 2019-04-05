function [the,position,ref]=FindminF_traj(P_end)
%% Find minimum required force. Note that the trajectory is fixed, only the joint configuration is unsettled.

%% Initialize related parameters
Pini=[0 200 0]; % right ankle's position w.r.t the world frame
the = [];
position=[];
ref=[];

%% Manually set three points for the trajectory, and Fit a parabola to it.
if Pini(3) > P_end(3)
    % if target position is lower than the initial position
    y0=[Pini(2) Pini(2)+100 P_end(2)-200];
    z0=[Pini(3) Pini(3)+50  P_end(3)];
else
    % if target position is higher than or horizontal to the initial position
    y0=[Pini(2) P_end(2)-300 P_end(2)-200]; % offset for sliding motion(why?)
    z0=[Pini(3) P_end(3)+50 P_end(3)];
end
p = polyfit(y0,z0,2);

%% For each discrete point on the trajectory, calculate the minimum needed force and corresponding joint configuration
%q_init = [0;0;0;0;0;0];
for y=Pini(2):5:P_end(2) % this is not the global variable, thus value is unchanged
    % obtain corresponding z from the fitted function
    z = p(1) * y .^2 + p(2) * y .^1 + p(3);
    if y > P_end(2)-200 
        % when the front leg reach the other side of the ditch, keep
        % shifting to spare room(200 mm) for the left leg to stand 
        z = P_end(3);
    end
    
    % look for minimum required force using optimization, for each discrete
    % point on the trajectory
    [q, Fmin] = Fmin_Position(y, z);
    %[q, Fmin, q_init] = Fmin_Position(y, z, q_init); % update the initial joint configuration for next step's optimization
    
    % record corresponding joint configuration
    the=[the; q];
    
    
end

