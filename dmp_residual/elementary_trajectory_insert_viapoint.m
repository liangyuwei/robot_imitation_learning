function ele_traj_struct = elementary_trajectory_insert_viapoint(ele_traj_struct, x_via, h_via, quat_or_not)
%% This function inserts via-points into elementary trajectory. Note that this is only used for modeling elementary trajectory, not for adding new via-points after f(x) is learned through DMP.    
% x_seq - canonical time
% y_seq - We need the original imitation data, to obtain correct vel and acc data;;;;used for interpolating to obtain accurate values of y(x0) and y(x1)
% f_seq - shape modulation data, used for interpolation


%% Find which interval x belongs to
if quat_or_not
    % quaternion data (simply add via-point quaternion to model)
    % store
    ele_traj_struct.quat = [ele_traj_struct.quat; h_via];
    ele_traj_struct.floor = [ele_traj_struct.floor, x_via];
    % sort the order
    [~, I] = sort(ele_traj_struct.floor);
    ele_traj_struct.floor = ele_traj_struct.floor(I);
    ele_traj_struct.quat = ele_traj_struct.quat(I, :);
else
    %% find floor and ceil
    id = find(x_via < ele_traj_struct.floor, 1); % id of ceiling
    if isempty(id)
        % the last point
        id = size(ele_traj_struct.floor, 2);
    end
    x0 = ele_traj_struct.floor(id-1);
    x1 = ele_traj_struct.floor(id); % x0 <= x_via < x1
    %% Prepare data for later solving equation for polynomial's coefficients
    a0 = ele_traj_struct.ab(id-1, 1); b0 = ele_traj_struct.ab(id-1, 2);
%     h_via = interp1(x_seq, y_seq, x_via) - interp1(x_seq, f_seq, x_via);
    h0_x0 = a0 * x0 + b0;
    h0_x1 = a0 * x1 + b0; % should it be from coefficients or from y-f ???? probably be from coefficients so as not to break the former result(h0(x0)==h(x0))
    a1 = (h_via - h0_x0) / (x_via - x0);
    b1 = h0_x0 - (h_via - h0_x0) / (x_via - x0) * x0;
    a2 = (h0_x1 - h_via) / (x1 - x_via);
    b2 = h_via - (h0_x1 - h_via) / (x1 - x_via) * x_via;

    
    %% Store the results
    % add 1 id to .floor, replace one line and add one line to .ab
    ele_traj_struct.floor = [ele_traj_struct.floor, x_via];
    ele_traj_struct.ab(id-1, :) = [a1, b1]; % replace the one closer to x0
    ele_traj_struct.ab = [ele_traj_struct.ab; [a2, b2]]; % add the one closer to x1
    % sort the order
    [~, I] = sort(ele_traj_struct.floor);
    ele_traj_struct.floor = ele_traj_struct.floor(I);
    ele_traj_struct.ab = ele_traj_struct.ab(I, :);
    
end

end