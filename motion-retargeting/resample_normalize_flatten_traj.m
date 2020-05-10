function resampled_normalized_traj = resample_normalize_flatten_traj(original_timestamps, original_traj, num_resampled_points)
%% This function resamples to same size and then normalizes to 0-1
% output: 1 x (DOF*num_resampled_poitns) row vector, the resampled, then normalized and finally flattened data   

% DOF to list of id
traj_id = sort(randperm(size(original_traj, 1)));
quat_id = [4,5,6,7, 14,15,16,17];
pos_angle_id = setdiff(traj_id, quat_id);

%% Resample through interpolation
% get ready
tmp_traj = original_traj; 
tmp_traj2 = zeros(size(tmp_traj, 1), num_resampled_points);
resampled_timestamps = linspace(0, original_timestamps(end), num_resampled_points); % resample 100 points
% - position and angle trajectories
% figure;
% plot3(tmp_traj(11, :), tmp_traj(12, :), tmp_traj(13, :), 'b-');
% hold on; grid on;
for i = 1:size(pos_angle_id, 2)
    % perform linear interpolation only on pos or angle data
    tmp_traj2(pos_angle_id(i), :) = interp1(original_timestamps, tmp_traj(pos_angle_id(i), :), resampled_timestamps, 'linear'); % 'spline'
end
% plot3(tmp_traj2(11, :), tmp_traj2(12, :), tmp_traj2(13, :), 'r.');

% - quaternion trajectories
quat_traj_l = original_traj(quat_id(1:4), :);
quat_traj_r = original_traj(quat_id(5:end), :);
quat_traj_l2 = zeros(4, num_resampled_points);  
quat_traj_r2 = zeros(4, num_resampled_points);
for i = 1 : num_resampled_points
    id = find((resampled_timestamps(i) <= original_timestamps), 1); % original_timestamps(id-1) < resampled_timestamps(i) <= original_timestamps(id)
    if id == 1 % actually outside of the original trajectory
        quat_traj_l2(:, i) = quat_traj_l(:, id);
        quat_traj_r2(:, i) = quat_traj_r(:, id);
    else
        t = (resampled_timestamps(i) - original_timestamps(id-1)) / (original_timestamps(id) - original_timestamps(id-1));
        quat_traj_l2(:, i) = slerp_matlab(quat_traj_l(:, id-1), quat_traj_l(:, id), t);
        quat_traj_r2(:, i) = slerp_matlab(quat_traj_r(:, id-1), quat_traj_r(:, id), t);
    end
end
% override
tmp_traj2(quat_id(1:4), :) = quat_traj_l2;
tmp_traj2(quat_id(5:end), :) = quat_traj_r2;


%% Normalize pos and angle to [0,1] (each element of unit quaternion is within [-1, 1], and the constraints between elements of quaternion should be protected from individual normalization)
tmp_traj3 = tmp_traj2;
for i = 1:size(pos_angle_id, 2)
    tmp_traj3(pos_angle_id(i), :) = mapminmax(tmp_traj2(pos_angle_id(i), :), 0, 1);
end
% figure;
% plot3(tmp_traj3(1, :), tmp_traj3(2, :), tmp_traj3(3, :), 'r.'); grid on;


%% Flatten 
% reshape is column-wise, tmp_traj3 is DOF x num_pathpoints now
tmp_traj4 = reshape(tmp_traj3, 1, size(tmp_traj3, 1) * size(tmp_traj3, 2));


resampled_normalized_traj = tmp_traj4;




end
