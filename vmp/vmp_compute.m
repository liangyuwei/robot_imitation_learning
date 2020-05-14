function y_seq = vmp_compute(ele_traj_struct, model, time_range, sigma_y, quat_or_not)
%% This function combines the elementary trajectory and shape modulation to output the final result.
% Input: ele_traj_struct - elementary trajectory;
%        model - shape modulation;
%        x or time_range ??? - we should probably do this one by one 

num_datapoints = size(time_range, 2);

%% The structure of elementary trajectory and shape modulation are already modified, so all that's need to do is to compute the function values.
if quat_or_not
    % quaternion data(4-dim result, 3-dim model)
    % elementary trajectory
    h_seq = zeros(4, num_datapoints);
    for n = 1:num_datapoints
%         h_seq(:, n) = elementary_trajectory_compute(ele_traj_struct, time_range(n), quat_or_not)';
        h_seq(:, n) = elementary_trajectory_compute_linear(ele_traj_struct, time_range(n), quat_or_not)';

    end
    
    % shape modulation
    f_seq = shape_modulation_compute(model, time_range, sigma_y, quat_or_not);
    % final
    y_seq = quatmultiply(h_seq', f_seq')';
    
else
    % pos or angle data(1-dim)
    % elementary trajectory
    h_seq = zeros(1, num_datapoints);
    for n = 1:num_datapoints
%         h_seq(n) = elementary_trajectory_compute(ele_traj_struct, time_range(n), quat_or_not);
        h_seq(n) = elementary_trajectory_compute_linear(ele_traj_struct, time_range(n), quat_or_not);

    end
    % shape modulation
    f_seq = shape_modulation_compute(model, time_range, sigma_y, quat_or_not);
    % final
    y_seq = h_seq + f_seq;
    
end


end