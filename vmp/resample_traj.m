function resampled_traj = resample_traj(original_timestamps, original_traj, num_resampled_points, quat_or_not)
%% This function resamples to same size and then normalizes to 0-1
% input: DOF x length
% output: DOF x num_resampled_points

DOF = size(original_traj, 1);

%% Resample through interpolation
resampled_timestamps = linspace(0, original_timestamps(end), num_resampled_points); % resample 100 points
% get ready
tmp_traj = original_traj;
tmp_traj2 = zeros(DOF, num_resampled_points);
if quat_or_not
    %% Quaternion data, through SLERP
    for i = 1 : num_resampled_points
        id = find((resampled_timestamps(i) <= original_timestamps), 1); % original_timestamps(id-1) < resampled_timestamps(i) <= original_timestamps(id)
        if id == 1 % actually outside of the original trajectory
            tmp_traj2(:, i) = tmp_traj(:, id);
        else
            t = (resampled_timestamps(i) - original_timestamps(id-1)) / (original_timestamps(id) - original_timestamps(id-1));
            tmp_traj2(:, i) = quatinterp(tmp_traj(:, id-1)', tmp_traj(:, id)', t)';%slerp_matlab(tmp_traj(:, id-1), tmp_traj(:, id), t);
        end
    end
else
    %% Position or Angle data, through linear interpolation
    % figure;
    % plot3(tmp_traj(11, :), tmp_traj(12, :), tmp_traj(13, :), 'b-');
    % hold on; grid on;
    for i = 1:DOF
        % perform linear interpolation only on pos or angle data
        tmp_traj2(i, :) = interp1(original_timestamps, tmp_traj(i, :), resampled_timestamps, 'linear'); % 'spline'
    end
    % plot3(tmp_traj2(11, :), tmp_traj2(12, :), tmp_traj2(13, :), 'r.');
    
end


%% Return
resampled_traj = tmp_traj2;




end
