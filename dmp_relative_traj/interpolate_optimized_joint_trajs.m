%% Interpolate the joint trajectories from g2o optimization


%% Prep
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity-dense_collision_checking.h5';

group_name = 'gun_2';

arm_traj_1 = h5read(file_name, ['/', group_name, '/arm_traj_1']);


%% Interpolate
[DOF, num_datapoints] = size(arm_traj_1);
interp_datapoints = 10*num_datapoints; %100;
arm_traj_1_linear = zeros(DOF, interp_datapoints);
arm_traj_1_spline = zeros(DOF, interp_datapoints);
arm_traj_1_pchip = zeros(DOF, interp_datapoints);
ori_timestamps = linspace(0, 1, num_datapoints);
interp_timestamps = linspace(0, 1, interp_datapoints);
for d = 1 : DOF
   arm_traj_1_linear(d, :) = interp1(ori_timestamps, arm_traj_1(d, :), interp_timestamps, 'linear');
   arm_traj_1_spline(d, :) = interp1(ori_timestamps, arm_traj_1(d, :), interp_timestamps, 'spline');
   arm_traj_1_pchip(d, :) = interp1(ori_timestamps, arm_traj_1(d, :), interp_timestamps, 'pchip'); % shape-preserving piecewise cubic interpolation    %'cubic');
end

% Store the interpolated results
h5create(file_name, ['/', group_name, '/arm_traj_1_linear'], size(arm_traj_1_linear));
h5write(file_name, ['/', group_name, '/arm_traj_1_linear'], arm_traj_1_linear);

h5create(file_name, ['/', group_name, '/arm_traj_1_spline'], size(arm_traj_1_spline));
h5write(file_name, ['/', group_name, '/arm_traj_1_spline'], arm_traj_1_spline);

h5create(file_name, ['/', group_name, '/arm_traj_1_pchip'], size(arm_traj_1_pchip));
h5write(file_name, ['/', group_name, '/arm_traj_1_pchip'], arm_traj_1_pchip);



