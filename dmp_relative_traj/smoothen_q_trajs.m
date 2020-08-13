%% smoothen_q_trajs


file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
group_name = 'fengren_1';
num_datapoints = 50;


%% Load q trajectories
arm_traj_1 = h5read(file_name, ['/', group_name, '/arm_traj_1']);

% do three different ways of interpolation
arm_traj_1_spline = zeros(size(arm_traj_1, 1), size(1:0.5:num_datapoints, 2));
arm_traj_1_linear = zeros(size(arm_traj_1, 1), size(1:0.5:num_datapoints, 2));
arm_traj_1_cubic = zeros(size(arm_traj_1, 1), size(1:0.5:num_datapoints, 2));

for i = 1 : size(arm_traj_1, 1)
    arm_traj_1_linear(i, :) = interp1(1:num_datapoints, arm_traj_1(i, :), 1:0.5:num_datapoints, 'linear'); 
    arm_traj_1_spline(i, :) = interp1(1:num_datapoints, arm_traj_1(i, :), 1:0.5:num_datapoints, 'spline');
    arm_traj_1_cubic(i, :) = interp1(1:num_datapoints, arm_traj_1(i, :), 1:0.5:num_datapoints, 'cubic');     
end





