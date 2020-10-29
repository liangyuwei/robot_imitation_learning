%% Validate the results of Position Scaling

clear;
clc;

%% Prep
ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
pos_scaling_file_name = '../motion-retargeting/mocap_ik_results_YuMi_pos_scaling.h5';

group_name = 'gun_2';

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

l_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/l_wrist_pos']);
r_wrist_pos_human = h5read(ori_file_name, ['/', group_name, '/r_wrist_pos']);
l_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/l_elbow_pos']);
r_elbow_pos_human = h5read(ori_file_name, ['/', group_name, '/r_elbow_pos']);

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
    end
end


% 2 - Position Scaling reference (one of three: 1. scaling w.r.t the center of shoulders; 2. scaling w.r.t parent; 3. scaling w.r.t. shoulder)
% absolute
l_wrist_pos_pos_scaling_ref = h5read(pos_scaling_file_name, ['/', group_name, '/scaled_l_wrist_pos']);
r_wrist_pos_pos_scaling_ref = h5read(pos_scaling_file_name, ['/', group_name, '/scaled_r_wrist_pos']);
l_elbow_pos_pos_scaling_ref = h5read(pos_scaling_file_name, ['/', group_name, '/scaled_l_elbow_pos']);
r_elbow_pos_pos_scaling_ref = h5read(pos_scaling_file_name, ['/', group_name, '/scaled_r_elbow_pos']);
% relative
lrw_pos_pos_scaling_ref = l_wrist_pos_pos_scaling_ref - r_wrist_pos_pos_scaling_ref;
lew_pos_pos_scaling_ref = l_elbow_pos_pos_scaling_ref - l_wrist_pos_pos_scaling_ref;
rew_pos_pos_scaling_ref = r_elbow_pos_pos_scaling_ref - r_wrist_pos_pos_scaling_ref;
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        l_wrist_pos_pos_scaling_ref(j, :) = mapminmax(l_wrist_pos_pos_scaling_ref(j, :), 0, 1);
        r_wrist_pos_pos_scaling_ref(j, :) = mapminmax(r_wrist_pos_pos_scaling_ref(j, :), 0, 1);
        l_elbow_pos_pos_scaling_ref(j, :) = mapminmax(l_elbow_pos_pos_scaling_ref(j, :), 0, 1);
        r_elbow_pos_pos_scaling_ref(j, :) = mapminmax(r_elbow_pos_pos_scaling_ref(j, :), 0, 1);
    end
end


% 3 - Position Scaling tracking results
% absolute
l_wrist_pos_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_l_wrist_pos']);
r_wrist_pos_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_r_wrist_pos']);
l_elbow_pos_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_l_elbow_pos']);
r_elbow_pos_pos_scaling = h5read(pos_scaling_file_name, ['/', group_name, '/actual_r_elbow_pos']);
% relative
lrw_pos_pos_scaling = l_wrist_pos_pos_scaling - r_wrist_pos_pos_scaling;
lew_pos_pos_scaling = l_elbow_pos_pos_scaling - l_wrist_pos_pos_scaling;
rew_pos_pos_scaling = r_elbow_pos_pos_scaling - r_wrist_pos_pos_scaling;
% normalize (absolute)
if (normalize_flag)
    for j = 1 : 3
        % absolute
        l_wrist_pos_pos_scaling(j, :) = mapminmax(l_wrist_pos_pos_scaling(j, :), 0, 1);
        r_wrist_pos_pos_scaling(j, :) = mapminmax(r_wrist_pos_pos_scaling(j, :), 0, 1);
        l_elbow_pos_pos_scaling(j, :) = mapminmax(l_elbow_pos_pos_scaling(j, :), 0, 1);
        r_elbow_pos_pos_scaling(j, :) = mapminmax(r_elbow_pos_pos_scaling(j, :), 0, 1);
    end
end


%% Display reference shifted from the original
% assert(normalize_flag==false, 'Turn off normalization flag for ploting the original trajectories and tracked trajectories!!!');
if (~normalize_flag)
    %% Reference trajectories compared to the original ones
    figure;
    p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
    p2 = plot3(l_wrist_pos_pos_scaling_ref(1, :), l_wrist_pos_pos_scaling_ref(2, :), l_wrist_pos_pos_scaling_ref(3, :), 'r--');
    
    plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); 
    plot3(r_wrist_pos_pos_scaling_ref(1, :), r_wrist_pos_pos_scaling_ref(2, :), r_wrist_pos_pos_scaling_ref(3, :), 'r--');
    
    plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-');
    plot3(l_elbow_pos_pos_scaling_ref(1, :), l_elbow_pos_pos_scaling_ref(2, :), l_elbow_pos_pos_scaling_ref(3, :), 'r--');
    
    plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-'); 
    plot3(r_elbow_pos_pos_scaling_ref(1, :), r_elbow_pos_pos_scaling_ref(2, :), r_elbow_pos_pos_scaling_ref(3, :), 'r--');

    xlabel('x'); ylabel('y'); zlabel('z');
    legend([p1(1), p2(1)], 'Original Human Trajs', 'Scaled Ref Trajs', 'Location', 'NorthEastOutside');
    title('Reference trajectories compared to the original ones');
    
    %% Reference trajectories compared to the original ones
    figure;
    p1 = plot3(l_wrist_pos_pos_scaling(1, :), l_wrist_pos_pos_scaling(2, :), l_wrist_pos_pos_scaling(3, :), 'g--'); hold on; grid on;
    p2 = plot3(l_wrist_pos_pos_scaling_ref(1, :), l_wrist_pos_pos_scaling_ref(2, :), l_wrist_pos_pos_scaling_ref(3, :), 'r-');
    
    plot3(r_wrist_pos_pos_scaling(1, :), r_wrist_pos_pos_scaling(2, :), r_wrist_pos_pos_scaling(3, :), 'g--'); 
    plot3(r_wrist_pos_pos_scaling_ref(1, :), r_wrist_pos_pos_scaling_ref(2, :), r_wrist_pos_pos_scaling_ref(3, :), 'r-');
    
    plot3(l_elbow_pos_pos_scaling(1, :), l_elbow_pos_pos_scaling(2, :), l_elbow_pos_pos_scaling(3, :), 'g--');
    plot3(l_elbow_pos_pos_scaling_ref(1, :), l_elbow_pos_pos_scaling_ref(2, :), l_elbow_pos_pos_scaling_ref(3, :), 'r-');
    
    plot3(r_elbow_pos_pos_scaling(1, :), r_elbow_pos_pos_scaling(2, :), r_elbow_pos_pos_scaling(3, :), 'g--'); 
    plot3(r_elbow_pos_pos_scaling_ref(1, :), r_elbow_pos_pos_scaling_ref(2, :), r_elbow_pos_pos_scaling_ref(3, :), 'r-');

    xlabel('x'); ylabel('y'); zlabel('z');
    legend([p1(1), p2(1)], 'Tracked Trajs', 'Scaled Ref Trajs', 'Location', 'NorthEastOutside');
    title('Actually tracked trajectories compared to the reference ones');
end


%% Display comparison of normalized trajectories in split view
figure;

subplot(2, 2, 1);
p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_wrist_pos_pos_scaling_ref(1, :), l_wrist_pos_pos_scaling_ref(2, :), l_wrist_pos_pos_scaling_ref(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling Ref', 'Location', 'NorthEastOutside');
title('Left Wrist');

subplot(2, 2, 2);
p1 = plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_wrist_pos_pos_scaling_ref(1, :), r_wrist_pos_pos_scaling_ref(2, :), r_wrist_pos_pos_scaling_ref(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling Ref', 'Location', 'NorthEastOutside');
title('Right Wrist');

subplot(2, 2, 3);
p1 = plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_elbow_pos_pos_scaling_ref(1, :), l_elbow_pos_pos_scaling_ref(2, :), l_elbow_pos_pos_scaling_ref(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling Ref', 'Location', 'NorthEastOutside');
title('Left Elbow');

subplot(2, 2, 4);
p1 = plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_elbow_pos_pos_scaling_ref(1, :), r_elbow_pos_pos_scaling_ref(2, :), r_elbow_pos_pos_scaling_ref(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling Ref', 'Location', 'NorthEastOutside');
title('Right Elbow');


%% Display comparison of normalized trajectories in split view
figure;

subplot(2, 2, 1);
p1 = plot3(l_wrist_pos_human(1, :), l_wrist_pos_human(2, :), l_wrist_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_wrist_pos_pos_scaling(1, :), l_wrist_pos_pos_scaling(2, :), l_wrist_pos_pos_scaling(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling', 'Location', 'NorthEastOutside');
title('Left Wrist');

subplot(2, 2, 2);
p1 = plot3(r_wrist_pos_human(1, :), r_wrist_pos_human(2, :), r_wrist_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_wrist_pos_pos_scaling(1, :), r_wrist_pos_pos_scaling(2, :), r_wrist_pos_pos_scaling(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling', 'Location', 'NorthEastOutside');
title('Right Wrist');

subplot(2, 2, 3);
p1 = plot3(l_elbow_pos_human(1, :), l_elbow_pos_human(2, :), l_elbow_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(l_elbow_pos_pos_scaling(1, :), l_elbow_pos_pos_scaling(2, :), l_elbow_pos_pos_scaling(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling', 'Location', 'NorthEastOutside');
title('Left Elbow');

subplot(2, 2, 4);
p1 = plot3(r_elbow_pos_human(1, :), r_elbow_pos_human(2, :), r_elbow_pos_human(3, :), 'b-'); hold on; grid on;
p2 = plot3(r_elbow_pos_pos_scaling(1, :), r_elbow_pos_pos_scaling(2, :), r_elbow_pos_pos_scaling(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
legend([p1(1), p2(1)], 'Human Traj', 'Position Scaling', 'Location', 'NorthEastOutside');
title('Right Elbow');


%% Tracking cost
l_wrist_cost = sum(sqrt(sum((l_wrist_pos_pos_scaling - l_wrist_pos_pos_scaling_ref) .^ 2)))
r_wrist_cost = sum(sqrt(sum((r_wrist_pos_pos_scaling - r_wrist_pos_pos_scaling_ref) .^ 2)))
l_elbow_cost = sum(sqrt(sum((l_elbow_pos_pos_scaling - l_elbow_pos_pos_scaling_ref) .^ 2)))
r_elbow_cost = sum(sqrt(sum((r_elbow_pos_pos_scaling - r_elbow_pos_pos_scaling_ref) .^ 2)))

