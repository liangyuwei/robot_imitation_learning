%% visualize the movement of wrists' local coordinates

clear;
clc;

addpath('./m_fcts/');
addpath('../vmp/'); % use resample_traj

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
file_name = '../motion-retargeting/mocap_ik_results_YuMi_g2o_similarity.h5';
group_name = 'fengren_1';


%% Load the imitation data (directly applicable to robots, i.e. pose data matches local coordinate frames of the robot)
time = h5read(ori_file_name, ['/', group_name, '/time']);
l_wrist_pos = h5read(ori_file_name, ['/', group_name, '/l_wrist_pos']);
l_elbow_pos = h5read(ori_file_name, ['/', group_name, '/l_elbow_pos']);
r_wrist_pos = h5read(ori_file_name, ['/', group_name, '/r_wrist_pos']);
r_elbow_pos = h5read(ori_file_name, ['/', group_name, '/r_elbow_pos']); % input should be DOF x length

l_wrist_ori = h5read(ori_file_name, ['/', group_name, '/l_wrist_ori']);
r_wrist_ori = h5read(ori_file_name, ['/', group_name, '/r_wrist_ori']);

num_datapoints = size(l_wrist_ori, 2);

l_wrist_rot = zeros(3, 3, num_datapoints);
r_wrist_rot = zeros(3, 3, num_datapoints);

% convert
for n = 1 : num_datapoints
    l_wrist_rot(:, :, n) = reshape(l_wrist_ori(:, n), 3, 3)';
    r_wrist_rot(:, :, n) = reshape(r_wrist_ori(:, n), 3, 3)';
end


%% Display the trajectories
figure;
arrow_length = 0.05;
for n = 1 : num_datapoints
    % refresh
    hold off;
    
    % pos traj
    plot3(l_wrist_pos(1, :), l_wrist_pos(2, :), l_wrist_pos(3, :), 'b--'); hold on; grid on;
    plot3(r_wrist_pos(1, :), r_wrist_pos(2, :), r_wrist_pos(3, :), 'b--'); 
    plot3(l_elbow_pos(1, :), l_elbow_pos(2, :), l_elbow_pos(3, :), 'b--'); 
    plot3(r_elbow_pos(1, :), r_elbow_pos(2, :), r_elbow_pos(3, :), 'b--');  % original imitation data
    title('Demonstrated Movement under robots'' local reference frames ');
    view(45, 45);
    xlabel('x'); ylabel('y'); zlabel('z');
    
    % draw local frames
    l_origin = l_wrist_pos(:, n);
    r_origin = r_wrist_pos(:, n);
    l_x = l_wrist_rot(:, :, n) * [arrow_length, 0, 0]';
    l_y = l_wrist_rot(:, :, n) * [0, arrow_length, 0]';
    l_z = l_wrist_rot(:, :, n) * [0, 0, arrow_length]';
    r_x = r_wrist_rot(:, :, n) * [arrow_length, 0, 0]';
    r_y = r_wrist_rot(:, :, n) * [0, arrow_length, 0]';
    r_z = r_wrist_rot(:, :, n) * [0, 0, arrow_length]';
    quiver3(l_origin(1), l_origin(2), l_origin(3), l_x(1), l_x(2), l_x(3), 'r');
    quiver3(l_origin(1), l_origin(2), l_origin(3), l_y(1), l_y(2), l_y(3), 'g');
    quiver3(l_origin(1), l_origin(2), l_origin(3), l_z(1), l_z(2), l_z(3), 'b');
    quiver3(r_origin(1), r_origin(2), r_origin(3), r_x(1), r_x(2), r_x(3), 'r');
    quiver3(r_origin(1), r_origin(2), r_origin(3), r_y(1), r_y(2), r_y(3), 'g');
    quiver3(r_origin(1), r_origin(2), r_origin(3), r_z(1), r_z(2), r_z(3), 'b');    
    
    pause;
    
end


%% Display skeletal movement



