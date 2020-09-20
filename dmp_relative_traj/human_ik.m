%% This script performs Human IK on motion capture data, i.e. convert motion capture position(or maybe orientation) trajectories to 7-DOF joint trajectories.


%% Prep
file_name = '../motion-retargeting/test_imi_data.h5'; % the original demonstrations that are not transformed to YuMi's local frames
group_name = 'kaoqin_2';


%% Load motion capture data
% position
l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_up_pos']);
r_shoulder_pos = h5read(file_name, ['/', group_name, '/r_up_pos']);
l_elbow_pos = h5read(file_name, ['/', group_name, '/l_fr_pos']);
r_elbow_pos = h5read(file_name, ['/', group_name, '/r_fr_pos']);
l_wrist_pos = h5read(file_name, ['/', group_name, '/l_hd_pos']);
r_wrist_pos = h5read(file_name, ['/', group_name, '/r_hd_pos']);

% quaternion, in (x,y,z,w) form
l_wrist_quat = h5read(file_name, ['/', group_name, '/l_hd_quat']);
r_wrist_quat = h5read(file_name, ['/', group_name, '/r_hd_quat']);
l_elbow_quat = h5read(file_name, ['/', group_name, '/l_fr_quat']);
r_elbow_quat = h5read(file_name, ['/', group_name, '/r_fr_quat']);

num_datapoints = size(l_wrist_pos, 2);

% convert quaternions to rotation matrices
l_wrist_rot = quat2rotm([l_wrist_quat(4, :); l_wrist_quat(1:3, :)]');
r_wrist_rot = quat2rotm([r_wrist_quat(4, :); r_wrist_quat(1:3, :)]');
l_elbow_rot = quat2rotm([l_elbow_quat(4, :); l_elbow_quat(1:3, :)]');
r_elbow_rot = quat2rotm([r_elbow_quat(4, :); r_elbow_quat(1:3, :)]');

%% Calculate 7-DOF joint angles from mocap data
l_joint_angle = zeros(7, num_datapoints);
r_joint_angle = zeros(7, num_datapoints);
for n = 1 : num_datapoints
    l_joint_angle(:, n) = convert_mocap_to_joint1(l_shoulder_pos(:, n), l_elbow_pos(:, n), l_wrist_pos(:, n), l_wrist_rot(:, :, n), true, l_elbow_rot(:, :, n));
    r_joint_angle(:, n) = convert_mocap_to_joint1(r_shoulder_pos(:, n), r_elbow_pos(:, n), r_wrist_pos(:, n), r_wrist_rot(:, :, n), false, r_elbow_rot(:, :, n));
end

%% Store the human IK results into file
h5create(file_name, ['/', group_name, '/l_joint_angles_ik'], size(l_joint_angle));
h5write(file_name, ['/', group_name, '/l_joint_angles_ik'], l_joint_angle);

h5create(file_name, ['/', group_name, '/r_joint_angles_ik'], size(r_joint_angle));
h5write(file_name, ['/', group_name, '/r_joint_angles_ik'], r_joint_angle);


