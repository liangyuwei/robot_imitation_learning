function [the,position,ref] = FindminF_swing_traj(P_end)
%% Find minimum required force. Note that the trajectory is fixed, only the joint configuration is unsettled.

%% Initialize related parameters
Pini=[0 0 0]; % left ankle's initial position
the=[];
position=[];
ref=[];

%% Fit a particular curve, and manually set three points for the trajectory, and Fit a parabola to it.
position_mid_point = (Pini + P_end) / 2;
position_matrix = [Pini; position_mid_point; P_end];
x0 = position_matrix(:, 1)' + [0, -50, 0];
y0 = position_matrix(:, 2)' + [0, -50, 0];
p = polyfit(y0,x0,2);

%% For each discrete point on the trajectory, calculate the minimum needed force and corresponding joint configuration
%tmp_y = [];
%tmp_x = [];
%tmp_z = [];
for y = Pini(2):5:P_end(2) % this is not the global variable, thus value is unchanged
   
    % obtain corresponding x from the fitted function
    x = p(1) * y .^2 + p(2) * y .^1 + p(3);
    
    % manually set z coordinate
    height = 50;
    if y/5 <= 92 % len/2
        % first half of the trajectory
        z = height / 92 * (y/5);
    else
        % second half of the trajectory, symmetric to the first half
        z = height - height / 92 * (y/5-92);
    end
    
    % look for minimum required force and corresponding joint configuration
    [q, ~] = Fmin_Swing_Position(x, y, z);
 
    % record corresponding joint configuration
    the = [the; q];
 
    % record data point for display
    %tmp_x = [tmp_x; x];
    %tmp_y = [tmp_y; y];
    %tmp_z = [tmp_z; z];
 
end
% visualize the trajectory
%{
plot3(tmp_x, tmp_y, tmp_z, 'b');
grid on;
%}


end