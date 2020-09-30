%% Validate the results of pure IK...

clear;
clc;

%% Prep
ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
pure_ik_file_name = '../motion-retargeting/test_imi_data_YuMi_pure_ik.h5';

group_name = 'fengren_1';%'kai_2';%'baozhu_1';%'fengren_1';

num_datapoints = 50;

normalize_flag = false;


%% Load data 
% 1 - human demonstrations
% absolute
l_shoulder_pos_human = h5read(ori_file_name, ['/', group_name, '/l_shoulder_pos']);
r_shoulder_pos_human = h5read(ori_file_name, ['/', group_name, '/r_shoulder_pos']);
human_shoulder_center = (l_shoulder_pos_human(:, 1) + r_shoulder_pos_human(:, 1)) / 2;

l_shoulder_pos_robot = [0.05355, 0.0725, 0.51492]';
r_shoulder_pos_robot = [0.05255, -0.0725, 0.51492]'; 
robot_shoulder_center = (l_shoulder_pos_robot + r_shoulder_pos_robot) / 2;
manual_offset = [0, 0, 0.65]'; % z = 0.45 manual offset

l_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/l_wrist_pos']) - human_shoulder_center + manual_offset;
r_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/r_wrist_pos']) - human_shoulder_center + manual_offset;
l_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/l_elbow_pos']) - human_shoulder_center + manual_offset;
r_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/r_elbow_pos']) - human_shoulder_center + manual_offset;

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


% 2 - Pure IK tracking results
% absolute
l_wrist_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/l_wrist_pos']);
r_wrist_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/r_wrist_pos']);
l_elbow_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/l_elbow_pos']);
r_elbow_pos_pure_ik = h5read(pure_ik_file_name, ['/', group_name, '/r_elbow_pos']);
% relative
lrw_pos_pure_ik = l_wrist_pos_pure_ik - r_wrist_pos_pure_ik;
lew_pos_pure_ik = l_elbow_pos_pure_ik - l_wrist_pos_pure_ik;
rew_pos_pure_ik = r_elbow_pos_pure_ik - r_wrist_pos_pure_ik;
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        l_wrist_pos_pure_ik(j, :) = mapminmax(l_wrist_pos_pure_ik(j, :), 0, 1);
        r_wrist_pos_pure_ik(j, :) = mapminmax(r_wrist_pos_pure_ik(j, :), 0, 1);
        l_elbow_pos_pure_ik(j, :) = mapminmax(l_elbow_pos_pure_ik(j, :), 0, 1);
        r_elbow_pos_pure_ik(j, :) = mapminmax(r_elbow_pos_pure_ik(j, :), 0, 1);
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
    p2 = plot3(l_wrist_pos_pure_ik(1, :), l_wrist_pos_pure_ik(2, :), l_wrist_pos_pure_ik(3, :), 'r-');
    
    plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); 
    plot3(r_wrist_pos_pure_ik(1, :), r_wrist_pos_pure_ik(2, :), r_wrist_pos_pure_ik(3, :), 'r-');
    
    plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-');
    plot3(l_elbow_pos_pure_ik(1, :), l_elbow_pos_pure_ik(2, :), l_elbow_pos_pure_ik(3, :), 'r-');
    
    plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-'); 
    plot3(r_elbow_pos_pure_ik(1, :), r_elbow_pos_pure_ik(2, :), r_elbow_pos_pure_ik(3, :), 'r-');

    xlabel('x'); ylabel('y'); zlabel('z');
    legend([p1(1), p2(1)], 'Shifted Human Traj', 'Pure IK Result', 'Location', 'NorthEastOutside');
    title('Shifted human trajectories and actually tracked trajectories');
end



%% Display comparison of normalized trajectories in split view
figure;

subplot(2, 2, 1);
p1 = plot3(l_wrist_pos_human_aligned(1, :), l_wrist_pos_human_aligned(2, :), l_wrist_pos_human_aligned(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_wrist_pos_pure_ik(1, :), l_wrist_pos_pure_ik(2, :), l_wrist_pos_pure_ik(3, :), 'r-');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Left Wrist');

subplot(2, 2, 2);
p1 = plot3(r_wrist_pos_human_aligned(1, :), r_wrist_pos_human_aligned(2, :), r_wrist_pos_human_aligned(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_wrist_pos_pure_ik(1, :), r_wrist_pos_pure_ik(2, :), r_wrist_pos_pure_ik(3, :), 'r-');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Right Wrist');

subplot(2, 2, 3);
p1 = plot3(l_elbow_pos_human_aligned(1, :), l_elbow_pos_human_aligned(2, :), l_elbow_pos_human_aligned(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_elbow_pos_pure_ik(1, :), l_elbow_pos_pure_ik(2, :), l_elbow_pos_pure_ik(3, :), 'r-');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Left Elbow');

subplot(2, 2, 4);
p1 = plot3(r_elbow_pos_human_aligned(1, :), r_elbow_pos_human_aligned(2, :), r_elbow_pos_human_aligned(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_elbow_pos_pure_ik(1, :), r_elbow_pos_pure_ik(2, :), r_elbow_pos_pure_ik(3, :), 'r-');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Pure IK', 'Location', 'NorthEastOutside');
title('Right Elbow');

% tracking error
l_wrist_pos_error = sum(sqrt(sum((l_wrist_pos_human_aligned - l_wrist_pos_pure_ik).^2)))
r_wrist_pos_error = sum(sqrt(sum((r_wrist_pos_human_aligned - r_wrist_pos_pure_ik).^2)))
l_elbow_pos_error = sum(sqrt(sum((l_elbow_pos_human_aligned - l_elbow_pos_pure_ik).^2)))
r_elbow_pos_error = sum(sqrt(sum((r_elbow_pos_human_aligned - r_elbow_pos_pure_ik).^2)))


