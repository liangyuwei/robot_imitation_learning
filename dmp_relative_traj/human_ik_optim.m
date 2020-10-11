%% This script performs Human IK on motion capture data, i.e. convert motion capture position(or maybe orientation) trajectories to 7-DOF joint trajectories.
% Using optimization to solve IK for more precise results!!!

%% This is used for providing data fed into Affine Deformation-based optimization. After the original motion capture data is converted into corresponding joint trajectories using this script, they should be processed by map_human_joints_to_yumi.m for the next step.


%% Prep
file_name = '../motion-retargeting/test_imi_data.h5'; % the original demonstrations that are not transformed to YuMi's local frames
group_name = 'zhenli_9';


%% Load motion capture data
% position
l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_up_pos']);
r_shoulder_pos = h5read(file_name, ['/', group_name, '/r_up_pos']);
l_elbow_pos = h5read(file_name, ['/', group_name, '/l_fr_pos']);
r_elbow_pos = h5read(file_name, ['/', group_name, '/r_fr_pos']);
l_wrist_pos = h5read(file_name, ['/', group_name, '/l_hd_pos']);
r_wrist_pos = h5read(file_name, ['/', group_name, '/r_hd_pos']);

% quaternion, in (x,y,z,w) form
l_shoulder_quat = h5read(file_name, ['/', group_name, '/l_up_quat']);
r_shoulder_quat = h5read(file_name, ['/', group_name, '/r_up_quat']);
l_elbow_quat = h5read(file_name, ['/', group_name, '/l_fr_quat']);
r_elbow_quat = h5read(file_name, ['/', group_name, '/r_fr_quat']);
l_wrist_quat = h5read(file_name, ['/', group_name, '/l_hd_quat']);
r_wrist_quat = h5read(file_name, ['/', group_name, '/r_hd_quat']);

num_datapoints = size(l_wrist_pos, 2);

% convert quaternions to rotation matrices
l_shoulder_rot = quat2rotm([l_shoulder_quat(4, :); l_shoulder_quat(1:3, :)]');
r_shoulder_rot = quat2rotm([r_shoulder_quat(4, :); r_shoulder_quat(1:3, :)]');
l_elbow_rot = quat2rotm([l_elbow_quat(4, :); l_elbow_quat(1:3, :)]');
r_elbow_rot = quat2rotm([r_elbow_quat(4, :); r_elbow_quat(1:3, :)]');
l_wrist_rot = quat2rotm([l_wrist_quat(4, :); l_wrist_quat(1:3, :)]');
r_wrist_rot = quat2rotm([r_wrist_quat(4, :); r_wrist_quat(1:3, :)]');

% for recording link lengths and shoulder position information
len_up_l = zeros(1, num_datapoints);
len_up_r = zeros(1, num_datapoints);
len_fr_l = zeros(1, num_datapoints);
len_fr_r = zeros(1, num_datapoints);
shoulder_pos_l = zeros(3, num_datapoints);
shoulder_pos_r = zeros(3, num_datapoints);


%% Calculate 7-DOF joint angles from mocap data
l_joint_angle = zeros(7, num_datapoints);
r_joint_angle = zeros(7, num_datapoints);

% optimization setup
ub = [pi, pi/2, pi*2/3, pi*2/3, pi*2/3, pi/2, pi/4];
lb = -ub; % the same as human model urdf setting
x0_l = zeros(7, 1); % initial value
x0_r = zeros(7, 1); % initial value

% global tracking_goal

% iterate to perform FK for each path point
l_cost_history = zeros(num_datapoints, 1);
r_cost_history = zeros(num_datapoints, 1);
for n = 1 : num_datapoints
    % debug information
    disp(['>>>> Tracking path point ', num2str(n), '/', num2str(num_datapoints), '...']);
    
    % 1 - Left
    disp('>> Left arm...');
    % get goals
    tracking_goal.shoulder_pos_goal = l_shoulder_pos(:, n);
    tracking_goal.elbow_pos_goal = l_elbow_pos(:, n);
    tracking_goal.wrist_pos_goal = l_wrist_pos(:, n); % position
    tracking_goal.shoulder_rot_goal = l_shoulder_rot(:, :, n);
    tracking_goal.elbow_rot_goal = l_elbow_rot(:, :, n);
    tracking_goal.wrist_rot_goal = l_wrist_rot(:, :, n); % orientation
    tracking_goal.upperarm_length = norm(tracking_goal.shoulder_pos_goal - tracking_goal.elbow_pos_goal);
    tracking_goal.forearm_length = norm(tracking_goal.elbow_pos_goal - tracking_goal.wrist_pos_goal);
    tracking_goal.left_or_right = true;
    % record for later use in visualization
    len_up_l(n) = tracking_goal.upperarm_length;
    len_fr_l(n) = tracking_goal.forearm_length;
    shoulder_pos_l(:, n) = tracking_goal.shoulder_pos_goal;
    % start optimization
    [xl, fl] = fmincon(@(x)human_ik_cost(x, tracking_goal), x0_l, [], [], [], [], lb, ub);
    x0_l = xl;
    l_cost_history(n) = fl;
    % store the result
    l_joint_angle(:, n) = xl;
    
    % 2 - Right
    disp('>> Right arm...');
     % get goals
    tracking_goal.shoulder_pos_goal = r_shoulder_pos(:, n);
    tracking_goal.elbow_pos_goal = r_elbow_pos(:, n);
    tracking_goal.wrist_pos_goal = r_wrist_pos(:, n); % position
    tracking_goal.shoulder_rot_goal = r_shoulder_rot(:, :, n);
    tracking_goal.elbow_rot_goal = r_elbow_rot(:, :, n);
    tracking_goal.wrist_rot_goal = r_wrist_rot(:, :, n); % orientation
    tracking_goal.upperarm_length = norm(tracking_goal.shoulder_pos_goal - tracking_goal.elbow_pos_goal);
    tracking_goal.forearm_length = norm(tracking_goal.elbow_pos_goal - tracking_goal.wrist_pos_goal);
    tracking_goal.left_or_right = false;
    % record for later use in visualization
    len_up_r(n) = tracking_goal.upperarm_length;
    len_fr_r(n) = tracking_goal.forearm_length;
    shoulder_pos_r(:, n) = tracking_goal.shoulder_pos_goal;
    % start optimization
    [xr, fr] = fmincon(@(x)human_ik_cost(x, tracking_goal), x0_r, [], [], [], [], lb, ub);
    x0_r = xr;
    r_cost_history(n) = fr;
    % store the result
    r_joint_angle(:, n) = xr;
end


%% Plot joint angle results
figure;
for i = 1 : 7
    subplot(2, 7, i);
    plot(1:num_datapoints, l_joint_angle(i, :), 'b-'); hold on; grid on;
    xlabel('Path point'); ylabel('Joint angle/radius');
end
for i = 1 : 7
    subplot(2, 7, i+7);
    plot(1:num_datapoints, r_joint_angle(i, :), 'r-'); hold on; grid on;
    xlabel('Path point'); ylabel('Joint angle/radius');
end
sgtitle('Left and right arm joint trajectories');


%% Check the corresponding actually executed 3d position trajectories
% initialization
shoulder_pos_tracked_l = zeros(3, num_datapoints);
shoulder_pos_tracked_r = zeros(3, num_datapoints);
elbow_pos_tracked_l = zeros(3, num_datapoints);
elbow_pos_tracked_r = zeros(3, num_datapoints);
wrist_pos_tracked_l = zeros(3, num_datapoints);
wrist_pos_tracked_r = zeros(3, num_datapoints);
wrist_eul_tracked_l = zeros(3, num_datapoints);
wrist_eul_tracked_r = zeros(3, num_datapoints);
wrist_rot_tracked_l = zeros(3, 3, num_datapoints);
wrist_rot_tracked_r = zeros(3, 3, num_datapoints);
% original data conversion
l_wrist_eul = zeros(3, num_datapoints);
r_wrist_eul = zeros(3, num_datapoints);

for i = 1 : num_datapoints
    % left
    [T_sld, T_elb, T_wri] = human_fk(l_shoulder_pos(:, i), l_joint_angle(:, i), len_up_l(i), len_fr_l(i), true, false);
    shoulder_pos_tracked_l(:, i) = T_sld(1:3, end);
    elbow_pos_tracked_l(:, i) = T_elb(1:3, end);
    wrist_pos_tracked_l(:, i) = T_wri(1:3, end);
    wrist_eul_tracked_l(:, i) = rotm2eul(T_wri(1:3, 1:3), 'XYZ');
    wrist_rot_tracked_l(:, :, i) = T_wri(1:3, 1:3);
    % right
    [T_sld, T_elb, T_wri] = human_fk(r_shoulder_pos(:, i), r_joint_angle(:, i), len_up_r(i), len_fr_r(i), false, false);
    shoulder_pos_tracked_r(:, i) = T_sld(1:3, end);
    elbow_pos_tracked_r(:, i) = T_elb(1:3, end);
    wrist_pos_tracked_r(:, i) = T_wri(1:3, end);    
    wrist_eul_tracked_r(:, i) = rotm2eul(T_wri(1:3, 1:3), 'XYZ');
    wrist_rot_tracked_r(:, :, i) = T_wri(1:3, 1:3);
    % conversion
    l_wrist_eul(:, i) = rotm2eul(l_wrist_rot(:, :, i), 'XYZ')';
    r_wrist_eul(:, i) = rotm2eul(r_wrist_rot(:, :, i), 'XYZ')';
end


figure;

subplot(3, 4, 1);
p1 = plot3(l_shoulder_pos(1, :), l_shoulder_pos(2, :), l_shoulder_pos(3, :), 'b-', 'LineWidth', 1); hold on; grid on;
p2 = plot3(shoulder_pos_tracked_l(1, :), shoulder_pos_tracked_l(2, :), shoulder_pos_tracked_l(3, :), 'r--', 'LineWidth', 2);
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Left Shoulder Pos');

subplot(3, 4, 2);
p1 = plot3(l_elbow_pos(1, :), l_elbow_pos(2, :), l_elbow_pos(3, :), 'b-'); hold on; grid on;
p2 = plot3(elbow_pos_tracked_l(1, :), elbow_pos_tracked_l(2, :), elbow_pos_tracked_l(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Left Elbow Pos');

subplot(3, 4, 3);
p1 = plot3(l_wrist_pos(1, :), l_wrist_pos(2, :), l_wrist_pos(3, :), 'b-'); hold on; grid on;
p2 = plot3(wrist_pos_tracked_l(1, :), wrist_pos_tracked_l(2, :), wrist_pos_tracked_l(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Left Wrist Pos');

subplot(3, 4, 4);
p1 = plot3(l_wrist_eul(1, :), l_wrist_eul(2, :), l_wrist_eul(3, :), 'b-'); hold on; grid on;
p2 = plot3(wrist_eul_tracked_l(1, :), wrist_eul_tracked_l(2, :), wrist_eul_tracked_l(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Left Wrist Eul');

subplot(3, 4, 5);
p1 = plot3(r_shoulder_pos(1, :), r_shoulder_pos(2, :), r_shoulder_pos(3, :), 'b-'); hold on; grid on;
p2 = plot3(shoulder_pos_tracked_r(1, :), shoulder_pos_tracked_r(2, :), shoulder_pos_tracked_r(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Right Shoulder Pos');

subplot(3, 4, 6);
p1 = plot3(r_elbow_pos(1, :), r_elbow_pos(2, :), r_elbow_pos(3, :), 'b-'); hold on; grid on;
p2 = plot3(elbow_pos_tracked_r(1, :), elbow_pos_tracked_r(2, :), elbow_pos_tracked_r(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Right Elbow Pos');

subplot(3, 4, 7);
p1 = plot3(r_wrist_pos(1, :), r_wrist_pos(2, :), r_wrist_pos(3, :), 'b-'); hold on; grid on;
p2 = plot3(wrist_pos_tracked_r(1, :), wrist_pos_tracked_r(2, :), wrist_pos_tracked_r(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Right Wrist Pos');

subplot(3, 4, 8);
p1 = plot3(r_wrist_eul(1, :), r_wrist_eul(2, :), r_wrist_eul(3, :), 'b-'); hold on; grid on;
p2 = plot3(wrist_eul_tracked_r(1, :), wrist_eul_tracked_r(2, :), wrist_eul_tracked_r(3, :), 'r--');
xlabel('x'); ylabel('y'); zlabel('z');
% legend([p1(1), p2(1)], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
title('Right Wrist Eul');

hL = subplot(3, 4, 12);
plot(1, nan, 1, nan, 'r'); set(hL, 'Visible', 'off')
poshL = get(hL, 'position');     % Getting its position
lgd = legend(hL, [p1, p2], 'Original Traj', 'Tracked Traj', 'Location', 'NorthEastOutside');
set(lgd, 'position', poshL);      % Adjusting legend's position
axis(hL, 'off');           



% Quantitative statistics
disp('>>>> Quantitative statistics: ');
disp( ['Left Shoulder Pos error: ', num2str(mean(sqrt(sum((l_shoulder_pos - shoulder_pos_tracked_l).^2))))] );
disp( ['Left Elbow Pos error: ', num2str(mean(sqrt(sum((l_elbow_pos - elbow_pos_tracked_l).^2))))] );
disp( ['Left Wrist Pos error: ', num2str(mean(sqrt(sum((l_wrist_pos - wrist_pos_tracked_l).^2))))] );
disp( ['Left Wrist Ori error(minimum angles between rotation matrices): ', num2str(mean_rot_error(l_wrist_rot, wrist_rot_tracked_l))]);
disp( ['Right Shoulder Pos error: ', num2str(mean(sqrt(sum((r_shoulder_pos - shoulder_pos_tracked_r).^2))))] );
disp( ['Right Elbow Pos error: ', num2str(mean(sqrt(sum((r_elbow_pos - elbow_pos_tracked_r).^2))))] );
disp( ['Right Wrist Pos error: ', num2str(mean(sqrt(sum((r_wrist_pos - wrist_pos_tracked_r).^2))))] );
disp( ['Right Wrist Ori error(minimum angles between rotation matrices): ', num2str(mean_rot_error(r_wrist_rot, wrist_rot_tracked_r))]);


%% Plot cost history statistics
figure;
plot(1:num_datapoints, l_cost_history, 'b-'); hold on; grid on;
plot(1:num_datapoints, r_cost_history, 'r-');
title('Cost history');
xlabel('Path point'); ylabel('Costs');


%% Display the joint trajectory
display_human_ik_movements(l_joint_angle, r_joint_angle, shoulder_pos_l, shoulder_pos_r, len_up_l, len_up_r, len_fr_l, len_fr_r);


%% Store the human IK results into file
h5create(file_name, ['/', group_name, '/l_joint_angles_optim_ik'], size(l_joint_angle));
h5write(file_name, ['/', group_name, '/l_joint_angles_optim_ik'], l_joint_angle);

h5create(file_name, ['/', group_name, '/r_joint_angles_optim_ik'], size(r_joint_angle));
h5write(file_name, ['/', group_name, '/r_joint_angles_optim_ik'], r_joint_angle);


