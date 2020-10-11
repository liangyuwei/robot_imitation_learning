%% Validate the results of Hujin's retargeting method

clear;
clc;

%% Prep
ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
hujin_file_name = '../motion-retargeting/mocap_ik_results_YuMi_hujin.h5';

group_name = 'zhenli_9';%'kai_2';%'baozhu_1';%'fengren_1';

num_datapoints = 50;

normalize_flag = false; % no need to normalize at all, since we are not comparing different methods!!!!


%% Load data 
% 1 - human demonstrations
% absolute
l_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/l_wrist_pos']);
r_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/r_wrist_pos']);
l_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/l_elbow_pos']);
r_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/r_elbow_pos']);

% way 2 - calculate offsets in the same way as we do
%{
le_set_start = [0.218, 0.310, 0.378]'; 
rw_set_start = [0.410, -0.179, 0.191]'; 
re_set_start = [0.218, -0.310, 0.377]';
lw_set_start = rw_set_start + (l_wrist_pos_human(:, 1) - r_wrist_pos_human(:, 1)); 
dist_y = abs(lw_set_start(2) - rw_set_start(2));
lw_set_start(2) = dist_y / 2.0;
rw_set_start(2) = -dist_y / 2.0;
lw_set_offset = lw_set_start - l_wrist_pos_human(:, 1);
le_set_offset = le_set_start - l_elbow_pos_human(:, 1);
rw_set_offset = rw_set_start - r_wrist_pos_human(:, 1);
re_set_offset = re_set_start - r_elbow_pos_human(:, 1);
% apply offsets
l_wrist_pos_human = l_wrist_pos_human + lw_set_offset;
l_elbow_pos_human = l_elbow_pos_human + le_set_offset;
r_wrist_pos_human = r_wrist_pos_human + rw_set_offset;
r_elbow_pos_human = r_elbow_pos_human + re_set_offset;
%}

% way 3 - shift the whole trajectories together as a whole (from center of shoulders to the origin of world frame)
%{
l_shoulder_pos_human = h5read(ori_file_name, ['/', group_name, '/l_shoulder_pos']);
r_shoulder_pos_human = h5read(ori_file_name, ['/', group_name, '/r_shoulder_pos']);
human_shoulder_center = (l_shoulder_pos_human(:, 1) + r_shoulder_pos_human(:, 1)) / 2;
manual_offset = [0.0, 0, 0.65]'; 
% apply offset
l_wrist_pos_human = l_wrist_pos_human - human_shoulder_center + manual_offset;
r_wrist_pos_human = r_wrist_pos_human - human_shoulder_center + manual_offset;
l_elbow_pos_human = l_elbow_pos_human - human_shoulder_center + manual_offset;
r_elbow_pos_human = r_elbow_pos_human - human_shoulder_center + manual_offset;
%}

% way 4 - shift the whole trajectories together as a whole (move human shoulder center to robot shoulder center)
%
l_shoulder_pos_human = h5read(ori_file_name, ['/', group_name, '/l_shoulder_pos']);
r_shoulder_pos_human = h5read(ori_file_name, ['/', group_name, '/r_shoulder_pos']);
human_shoulder_center = (l_shoulder_pos_human(:, 1) + r_shoulder_pos_human(:, 1)) / 2;
robot_shoulder_center = ([0.05355, 0.0725, 0.51492]' + [0.05355, -0.0725, 0.51492]') / 2;
manual_offset = [0.25, 0.0, 0.0]'; %[0, 0, 0]'; %
human_robot_offset = robot_shoulder_center - human_shoulder_center + manual_offset; 
% apply offset
l_wrist_pos_human = l_wrist_pos_human + human_robot_offset;
r_wrist_pos_human = r_wrist_pos_human + human_robot_offset;
l_elbow_pos_human = l_elbow_pos_human + human_robot_offset;
r_elbow_pos_human = r_elbow_pos_human + human_robot_offset;
%}


% relative
lrw_pos_human = l_wrist_pos_human - r_wrist_pos_human;
lew_pos_human = l_elbow_pos_human - l_wrist_pos_human;
rew_pos_human = r_elbow_pos_human - r_wrist_pos_human;
% normalize to [0,1] x [0,1] x [0,1]
if (normalize_flag)
    for i = 1 : 3
        % absolute
        l_wrist_pos_human(i, :) = mapminmax(l_wrist_pos_human(i, :), 0, 1);
        r_wrist_pos_human(i, :) = mapminmax(r_wrist_pos_human(i, :), 0, 1);
        l_elbow_pos_human(i, :) = mapminmax(l_elbow_pos_human(i, :), 0, 1);
        r_elbow_pos_human(i, :) = mapminmax(r_elbow_pos_human(i, :), 0, 1);
        % relative
    %     lrw_pos_human(i, :) = mapminmax(lrw_pos_human(i, :), 0, 1);
    %     lew_pos_human(i, :) = mapminmax(lew_pos_human(i, :), 0, 1);
    %     rew_pos_human(i, :) = mapminmax(rew_pos_human(i, :), 0, 1);
    end
end
% align the length
%{
l_wrist_pos_human_aligned = zeros(3, num_datapoints);
l_elbow_pos_human_aligned = zeros(3, num_datapoints);
r_wrist_pos_human_aligned = zeros(3, num_datapoints);
r_elbow_pos_human_aligned = zeros(3, num_datapoints);
lrw_pos_human_aligned = zeros(3, num_datapoints);
lew_pos_human_aligned = zeros(3, num_datapoints);
rew_pos_human_aligned = zeros(3, num_datapoints);
original_num_datapoints = size(l_wrist_pos_human, 2);
original_timestamps = linspace(0, 1, original_num_datapoints);
target_timestamps = linspace(0, 1, num_datapoints);
for i = 1 : 3
    l_wrist_pos_human_aligned(i, :) = interp1(original_timestamps, l_wrist_pos_human(i, :), target_timestamps, 'linear');
    l_elbow_pos_human_aligned(i, :) = interp1(original_timestamps, l_elbow_pos_human(i, :), target_timestamps, 'linear');
    r_wrist_pos_human_aligned(i, :) = interp1(original_timestamps, r_wrist_pos_human(i, :), target_timestamps, 'linear');
    r_elbow_pos_human_aligned(i, :) = interp1(original_timestamps, r_elbow_pos_human(i, :), target_timestamps, 'linear');
    lrw_pos_human_aligned(i, :) = interp1(original_timestamps, lrw_pos_human(i, :), target_timestamps, 'linear');
    lew_pos_human_aligned(i, :) = interp1(original_timestamps, lew_pos_human(i, :), target_timestamps, 'linear');
    rew_pos_human_aligned(i, :) = interp1(original_timestamps, rew_pos_human(i, :), target_timestamps, 'linear');
end
%}


% 2 - Hujin's affine optim tracking results
% absolute
l_wrist_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_l_wrist_pos_traj_hujin']); %'/ref_l_wrist_traj_hujin']);%'/actual_l_wrist_pos_traj_hujin_original']); %
r_wrist_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_r_wrist_pos_traj_hujin']); %'/ref_r_wrist_traj_hujin']);%'/actual_r_wrist_pos_traj_hujin_original']); %
l_elbow_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_l_elbow_pos_traj_hujin']); %'/ref_l_elbow_traj_hujin']);%'/actual_l_elbow_pos_traj_hujin_original']); %
r_elbow_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/actual_r_elbow_pos_traj_hujin']); %'/ref_r_elbow_traj_hujin']);%'/actual_r_elbow_pos_traj_hujin_original']); %
%
% l_wrist_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/ref_l_wrist_traj_hujin']);%'/actual_l_wrist_pos_traj_hujin_original']); %
% r_wrist_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/ref_r_wrist_traj_hujin']);%'/actual_r_wrist_pos_traj_hujin_original']); %
% l_elbow_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/ref_l_elbow_traj_hujin']);%'/actual_l_elbow_pos_traj_hujin_original']); %
% r_elbow_pos_hujin = h5read(hujin_file_name, ['/', group_name, '/ref_r_elbow_traj_hujin']);%'/actual_r_elbow_pos_traj_hujin_original']); %
% relative
lrw_pos_hujin = l_wrist_pos_hujin - r_wrist_pos_hujin;
lew_pos_hujin = l_elbow_pos_hujin - l_wrist_pos_hujin;
rew_pos_hujin = r_elbow_pos_hujin - r_wrist_pos_hujin;
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        l_wrist_pos_hujin(j, :) = mapminmax(l_wrist_pos_hujin(j, :), 0, 1);
        r_wrist_pos_hujin(j, :) = mapminmax(r_wrist_pos_hujin(j, :), 0, 1);
        l_elbow_pos_hujin(j, :) = mapminmax(l_elbow_pos_hujin(j, :), 0, 1);
        r_elbow_pos_hujin(j, :) = mapminmax(r_elbow_pos_hujin(j, :), 0, 1);
        % relative
%         actual_lrw_pos_traj(j, :) = mapminmax(actual_lrw_pos_traj(j, :), 0, 1);
%         actual_lew_pos_traj(j, :) = mapminmax(actual_lew_pos_traj(j, :), 0, 1);
%         actual_rew_pos_traj(j, :) = mapminmax(actual_rew_pos_traj(j, :), 0, 1);
    end
end


%% Display reference shifted from the original(the center of human shoulder positions moved to the origin of world, then lifted up 0.65 along z-axis)
% assert(normalize_flag==false, 'Turn off normalization flag for ploting the original trajectories and tracked trajectories!!!');
if (~normalize_flag)
    figure;
    p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
    p2 = plot3(l_wrist_pos_hujin(1, :), l_wrist_pos_hujin(2, :), l_wrist_pos_hujin(3, :), 'r--');
    
    plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); 
    plot3(r_wrist_pos_hujin(1, :), r_wrist_pos_hujin(2, :), r_wrist_pos_hujin(3, :), 'r--');
    
    plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-');
    plot3(l_elbow_pos_hujin(1, :), l_elbow_pos_hujin(2, :), l_elbow_pos_hujin(3, :), 'r--');
    
    plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-'); 
    plot3(r_elbow_pos_hujin(1, :), r_elbow_pos_hujin(2, :), r_elbow_pos_hujin(3, :), 'r--');

    xlabel('x'); ylabel('y'); zlabel('z');
    legend([p1(1), p2(1)], 'Reference Traj', 'Affine Optim Result', 'Location', 'NorthEastOutside');
    title('Reference trajectories and actually tracked trajectories');
end


%% Display comparison of normalized trajectories in split view
figure;

subplot(2, 2, 1);
p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_wrist_pos_hujin(1, :), l_wrist_pos_hujin(2, :), l_wrist_pos_hujin(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Left Wrist');

subplot(2, 2, 2);
p1 = plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_wrist_pos_hujin(1, :), r_wrist_pos_hujin(2, :), r_wrist_pos_hujin(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Right Wrist');

subplot(2, 2, 3);
p1 = plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_elbow_pos_hujin(1, :), l_elbow_pos_hujin(2, :), l_elbow_pos_hujin(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Left Elbow');

subplot(2, 2, 4);
p1 = plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_elbow_pos_hujin(1, :), r_elbow_pos_hujin(2, :), r_elbow_pos_hujin(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Right Elbow');

% tracking error
l_wrist_pos_error = sum(sqrt(sum((l_wrist_pos_human - l_wrist_pos_hujin).^2)))
r_wrist_pos_error = sum(sqrt(sum((r_wrist_pos_human - r_wrist_pos_hujin).^2)))
l_elbow_pos_error = sum(sqrt(sum((l_elbow_pos_human - l_elbow_pos_hujin).^2)))
r_elbow_pos_error = sum(sqrt(sum((r_elbow_pos_human - r_elbow_pos_hujin).^2)))

