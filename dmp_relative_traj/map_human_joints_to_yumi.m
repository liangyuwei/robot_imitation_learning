%% direct mapping from human IK results(joint trajectories) to robot by T-pose or I-pose..


%% Load human joint data
file_name = '../motion-retargeting/test_imi_data.h5'; % the original demonstrations that are not transformed to YuMi's local frames
group_name = 'fengren_1';
l_joint_angles_ik = h5read(file_name, ['/', group_name, '/l_joint_angles_optim_ik']); %'/l_joint_angles_ik']);
r_joint_angles_ik = h5read(file_name, ['/', group_name, '/r_joint_angles_optim_ik']); %'/r_joint_angles_ik']);


%% Prep
% joint mapping to cope with pairing and direction
k_human_yumi_l = [0, -1, 0, 0, 0, 0, 0; ...
                  1,  0, 0, 0, 0, 0, 0; ...
                  0, 0, -1, 0, 0, 0, 0; ...
                  0, 0, 0, 1, 0, 0, 0; ...
                  0, 0, 0, 0, -1, 0, 0; ...
                  0, 0, 0, 0, 0, 1, 0; ...
                  0, 0, 0, 0, 0, 0, 0]; % e.g. YuMi's q1 corresponds to Human's a2 joint, in the opposite direction; 
                  % Human's a7 has not corresponding joints in human
k_human_yumi_r = [0, 1, 0, 0, 0, 0, 0; ...
                 -1, 0, 0, 0, 0, 0, 0; ...
                  0, 0, -1, 0, 0, 0, 0; ...
                  0, 0, 0, 1, 0, 0, 0; ...
                  0, 0, 0, 0, -1, 0, 0; ...
                  0, 0, 0, 0, 0, 1, 0; ...
                  0, 0, 0, 0, 0, 0, 0];
                  % Human's a7 has not corresponding joints in human

% T-pose
% d_human_t_pose_l = [0, -pi/2, 0, 0, 0, 0, 0];
% d_human_t_pose_r = [0, -pi/2, 0, 0, 0, 0, 0];
% d_yumi_t_pose_l = [-0.8, -1.25, 2, -1.5, 0, 0, 0];
% d_yumi_t_pose_r = [0.8, -1.25, -2, -1.5, 0, 0, 0];

% "I-pose" (both arms straight down)
d_human_i_pose_l = [0, 0, 0, 0, 0, 0, 0];
d_human_i_pose_r = [0, 0, 0, 0, 0, 0, 0];
d_yumi_i_pose_l = [-2, -2,  2, -pi/2, 0, 0, 0]; %[-2, -1.95, 2, -1.5, 0, 0, 0];
d_yumi_i_pose_r = [ 2, -2, -2, -pi/2, 0, 0, 0]; %[2, -1.95, -2, -1.5, 0, 0, 0];


%% Transform to YuMi
l_joint_angles_ik_yumi = zeros(size(l_joint_angles_ik));
r_joint_angles_ik_yumi = zeros(size(r_joint_angles_ik));
num_datapoints = size(l_joint_angles_ik, 2);
% select same pose to sync
d_human_l = d_human_i_pose_l;
d_human_r = d_human_i_pose_r;
d_yumi_l = d_yumi_i_pose_l;
d_yumi_r = d_yumi_i_pose_r;
for n = 1 : num_datapoints
    % left
    l_joint_angles_ik_yumi(:, n) = k_human_yumi_l * (l_joint_angles_ik(:, n) - d_human_l') + d_yumi_l';
    % right
    r_joint_angles_ik_yumi(:, n) = k_human_yumi_r * (r_joint_angles_ik(:, n) - d_human_r') + d_yumi_r';        
end


%% Clamping to YuMi's corresponding joint angle range (these corresponds to {ub/lb}_{l/r} in human_ik.m)
ub_yumi_l = [0, 0.76, 2.94, 1.4, 3.15, pi/2, 0];
lb_yumi_l = [-2.94, -2.5, -0.9, -1.6, -pi/2, -pi/2, 0];
ub_yumi_r = [2.94, 0.76, 0.9, 1.4, pi/2, pi/2, 0];
lb_yumi_r = [0, -2.5, -2.94, -1.6, -3.15, -pi/2, 0];

for n = 1 : num_datapoints
   for d = 1 : 6 % no joint on YuMi corresponds to human's a7 joint!!! (and thus it's manually set to initial pose)
        % left
        l_joint_angles_ik_yumi(d, n) = max(min(l_joint_angles_ik_yumi(d, n), ub_yumi_l(d)), lb_yumi_l(d));
        % right
        r_joint_angles_ik_yumi(d, n) = max(min(r_joint_angles_ik_yumi(d, n), ub_yumi_r(d)), lb_yumi_r(d));
   end
end


%% Store the results
h5create(file_name, ['/', group_name, '/l_joint_angles_optim_ik_yumi'], size(l_joint_angles_ik_yumi));
h5write(file_name, ['/', group_name, '/l_joint_angles_optim_ik_yumi'], l_joint_angles_ik_yumi);

h5create(file_name, ['/', group_name, '/r_joint_angles_optim_ik_yumi'], size(r_joint_angles_ik_yumi));
h5write(file_name, ['/', group_name, '/r_joint_angles_optim_ik_yumi'], r_joint_angles_ik_yumi);


