%% Read and process csv motion-capture data, exported from Motive:Tracker software.
% The world frame constructed in motion-capture system is y-up frame, requiring a transformation during post-processing.    
% Empty frame should be dumped.
% Perform Kalmann Filter to smooth the trajectory...


%% Read .csv mocap data
file_name = 'point_motion.csv'; %'LeftHandInitState.csv';
L_UP_ID = 3; % upperarm
L_FR_ID = 23; % forearm
L_HD_ID = 47; % hand
data = csvread(file_name, 7, 0); % pure numeric data(frame, time, rigid boy quaternion/position, markers)  
% quaternion is read in as (w,x,y,z); pos is (x,y,z) with 'meters' as units.
% note: even at different initial locations, the pos is w.r.t the world frame determined during calibration!!!  
% note: initial frames are parallel to the world frame.
left_upperarm = struct('quat', [data(:, L_UP_ID+3), data(:, L_UP_ID:L_UP_ID+2)], 'pos', data(:, L_UP_ID+4:L_UP_ID+6)); % (x,y,z,w), read in as (w,x,y,z)
left_forearm = struct('quat', [data(:, L_FR_ID+3), data(:, L_FR_ID:L_FR_ID+2)], 'pos', data(:, L_FR_ID+4:L_FR_ID+6));
left_hand = struct('quat', [data(:, L_HD_ID+3), data(:, L_HD_ID:L_HD_ID+2)], 'pos', data(:, L_HD_ID+4:L_HD_ID+6));
time = data(:, 2);


%% Post-processing
% delete empty frame
empty_id = [];
for i = 1 : size(data, 1)
    if(sum(left_upperarm.quat(i, :))==0 || sum(left_upperarm.pos(i, :))==0 || ...
       sum(left_forearm.quat(i, :))==0 || sum(left_forearm.pos(i, :))==0 || ...
       sum(left_hand.quat(i, :))==0 || sum(left_hand.pos(i, :))==0)
        empty_id = [empty_id, i];
    end
end
disp(['The frame that has lost information: ', num2str(empty_id)]);
left_upperarm.quat(empty_id, :) = [];
left_upperarm.pos(empty_id, :) = [];
left_forearm.quat(empty_id, :) = [];
left_forearm.pos(empty_id, :) = [];
left_hand.quat(empty_id, :) = [];
left_hand.pos(empty_id, :) = [];
time(empty_id, :) = [];
data(empty_id, :) = [];


% convert from y-up frame to z-up frame
quat_shift = eul2quat([pi, -pi/2, -pi/2], 'ZYZ'); % (w,x,y,z)
for i = 1 : size(data, 1)
    % prepare
    l_up_quat = quatmultiply(quat_shift, left_upperarm.quat(i, :));
    l_fr_quat = quatmultiply(quat_shift, left_forearm.quat(i, :));
    l_hd_quat = quatmultiply(quat_shift, left_hand.quat(i, :)); 
    l_up_pos = (quat2rotm(quat_shift)*left_upperarm.pos(i, :)')';
    l_fr_pos = (quat2rotm(quat_shift)*left_forearm.pos(i, :)')';
    l_hd_pos = (quat2rotm(quat_shift)*left_hand.pos(i, :)')';
    % replace
    left_upperarm.quat(i, :) = l_up_quat;
    left_upperarm.pos(i, :) = l_up_pos;
    left_forearm.quat(i, :) = l_fr_quat;
    left_forearm.pos(i, :) = l_fr_pos;    
    left_hand.quat(i, :) = l_hd_quat;
    left_hand.pos(i, :) = l_hd_pos;  
end


% Kalmann Filter smoothing



%% Visualization(consists of transformation to z-up frame)
%{
figure;
grid on;
quat_ori = [1, 0, 0, 0]; % [w,x,y,z]
pos_ori = [0, 0, 0];

for i = 1:round(size(data, 1))

    % prepare parameters
    quat_shift = eul2quat([pi, -pi/2, -pi/2], 'ZYZ'); % (w,x,y,z)
    l_up_quat = quatmultiply(quat_shift, left_upperarm.quat(i, :));
    l_fr_quat = quatmultiply(quat_shift, left_forearm.quat(i, :));
    l_hd_quat = quatmultiply(quat_shift, left_hand.quat(i, :)); 
    l_up_pos = (quat2rotm(quat_shift)*left_upperarm.pos(i, :)')';
    l_fr_pos = (quat2rotm(quat_shift)*left_forearm.pos(i, :)')';
    l_hd_pos = (quat2rotm(quat_shift)*left_hand.pos(i, :)')';

    % draw
    draw_segment(quat_ori, pos_ori); % orientation of origin
    % plot settings
    axis([-1, 1, -1, 1, 0, 2]);
    xlabel('x'); ylabel('y'); zlabel('z');
    view(60,25);
    hold on; % disable refreshing
    
    draw_segment(l_up_quat, l_up_pos); % orientation of upper arm
    plot3([l_up_pos(1), l_fr_pos(1)], [l_up_pos(2), l_fr_pos(2)], [l_up_pos(3), l_fr_pos(3)], 'k-'); % connect upper arm and forearm  
    draw_segment(l_fr_quat, l_fr_pos); % orientation of forearm
    plot3([l_fr_pos(1), l_hd_pos(1)], [l_fr_pos(2), l_hd_pos(2)], [l_fr_pos(3), l_hd_pos(3)], 'k-'); % connect forearm and hand
    draw_segment(l_hd_quat, l_hd_pos); % orientation of hand
    hold off; % enable refreshing

    % pause
    pause(0.01);
    
end
%}

%% Convert to a whole
array_length = 27; % rotation matrices for three links
left_up_fr_hd_rot_path_1 = zeros(size(data, 1), array_length);
for i = 1 : size(data, 1)
    rotm_up = quat2rotm(left_upperarm.quat(i, :));
    rotm_fr = quat2rotm(left_forearm.quat(i, :));
    rotm_hd = quat2rotm(left_hand.quat(i, :));
    left_up_fr_hd_rot_path_1(i, 1:9) = reshape(rotm_up', [1,9]);
    left_up_fr_hd_rot_path_1(i, 10:18) = reshape(rotm_fr', [1,9]);
    left_up_fr_hd_rot_path_1(i, 19:27) = reshape(rotm_hd', [1,9]);
end


%% Write data to .h5
% h5create('mocap_paths.h5', 'left_upperarm_path_1', size([left_upperarm.pos, left_upperarm.quat])); % create before writing
% h5create('mocap_paths.h5', 'left_forearm_path_1', size([left_forearm.pos, left_forearm.quat]));
% h5create('mocap_paths.h5', 'left_hand_path_1', size([left_hand.pos, left_hand.quat]));
% h5write('mocap_paths.h5', 'left_upperarm_path_1', [left_upperarm.pos, left_upperarm.quat]);
% h5write('mocap_paths.h5', 'left_forearm_path_1', [left_forearm.pos, left_forearm.quat]);
% h5write('mocap_paths.h5', 'left_hand_path_1', [left_hand.pos, left_hand.quat]);
h5create('mocap_paths.h5', '/left_up_fr_hd_rot_path_1', size(left_up_fr_hd_rot_path_1'));
h5write('mocap_paths.h5', '/left_up_fr_hd_rot_path_1', left_up_fr_hd_rot_path_1');

