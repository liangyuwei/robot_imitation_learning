%% Read and process csv motion-capture data, exported from Motive:Tracker software.
% The world frame constructed in motion-capture system is y-up frame, requiring a transformation during post-processing.    
% Empty frame should be dumped.
% Perform Kalmann Filter to smooth the trajectory...


%% Read .csv mocap data
file_name = 'sweep_motion.csv'; %'circle_motion.csv'; %'sweep_motion.csv'; %'pull_motion.csv'; %'point_motion.csv'; %'LeftHandInitState.csv';
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
%{
quat_shift = eul2quat([pi, -pi/2, -pi/2], 'ZYZ'); % (w,x,y,z)
for i = 1 : size(data, 1)
    % prepare
    l_up_quat = quatmultiply(quat_shift, left_upperarm.quat(i, :));
    l_fr_quat = quatmultiply(quat_shift, left_forearm.quat(i, :));
    l_hd_quat = quatmultiply(quat_shift, left_hand.quat(i, :)); 
    l_up_pos = (quat2rotm(quat_shift)*left_upperarm.pos(i, :)')';
    l_fr_pos = (quat2rotm(quat_shift)*left_forearm.pos(i, :)')';
    l_hd_pos = (quat2rotm(quat_shift)*left_hand.pos(i, :)')';
    % convert to RViz UR5 frame of reference
    quat_up_shift_rviz = eul2quat([pi/2, 0, 0], 'ZYZ');
    quat_fr_shift_rviz = eul2quat([pi/2, 0, 0], 'ZYZ');
    quat_hd_shift_rviz = eul2quat([pi/2, pi/2, pi], 'ZYZ');
    l_up_quat_rviz = quatmultiply(quat_up_shift_rviz, l_up_quat);
    l_fr_quat_rviz = quatmultiply(quat_fr_shift_rviz, l_fr_quat);
    l_hd_quat_rviz = quatmultiply(quat_hd_shift_rviz, l_hd_quat);
    % replace
    left_upperarm.quat(i, :) = l_up_quat_rviz; %l_up_quat;
    left_upperarm.pos(i, :) = l_up_pos;
    left_forearm.quat(i, :) = l_fr_quat_rviz; %l_fr_quat;
    left_forearm.pos(i, :) = l_fr_pos;    
    left_hand.quat(i, :) = l_hd_quat_rviz; %l_hd_quat;
    left_hand.pos(i, :) = l_hd_pos;  
end
%}

% Kalmann Filter smoothing



%% Visualization(consists of transformation to z-up frame)
%
figure;
grid on;
quat_ori = [1, 0, 0, 0]; % [w,x,y,z]
pos_ori = [0, 0, 0];
record = false; %true;
gif_name = 'sweep_motion.gif';

for i = 1:floor(size(data, 1)/10)

    % prepare parameters
    quat_shift = eul2quat([pi, -pi/2, -pi/2], 'ZYZ'); % (w,x,y,z)
    l_up_quat = quatmultiply(quat_shift, left_upperarm.quat(i*10, :));
    l_fr_quat = quatmultiply(quat_shift, left_forearm.quat(i*10, :));
    l_hd_quat = quatmultiply(quat_shift, left_hand.quat(i*10, :)); 
    l_up_pos = (quat2rotm(quat_shift)*left_upperarm.pos(i*10, :)')';
    l_fr_pos = (quat2rotm(quat_shift)*left_forearm.pos(i*10, :)')';
    l_hd_pos = (quat2rotm(quat_shift)*left_hand.pos(i*10, :)')';
    
    quat_up_shift_rviz = eul2quat([pi/2, 0, 0], 'ZYZ');
    quat_fr_shift_rviz = eul2quat([pi/2, 0, 0], 'ZYZ');
    quat_hd_shift_rviz = eul2quat([pi/2, pi/2, pi], 'ZYZ');
    l_up_quat_rviz = quatmultiply(quat_up_shift_rviz, l_up_quat);
    l_fr_quat_rviz = quatmultiply(quat_fr_shift_rviz, l_fr_quat);
    l_hd_quat_rviz = quatmultiply(quat_hd_shift_rviz, l_hd_quat);

    % draw
    draw_segment(quat_ori, pos_ori); % orientation of origin
    % plot settings
    axis([-1, 1, -1, 1, 0, 2]);
    xlabel('x'); ylabel('y'); zlabel('z');
    view(60,25);
    hold on; % disable refreshing
    
%     draw_segment(l_up_quat, l_up_pos); % orientation of upper arm
%     plot3([l_up_pos(1), l_fr_pos(1)], [l_up_pos(2), l_fr_pos(2)], [l_up_pos(3), l_fr_pos(3)], 'k-'); % connect upper arm and forearm  
%     draw_segment(l_fr_quat, l_fr_pos); % orientation of forearm
%     plot3([l_fr_pos(1), l_hd_pos(1)], [l_fr_pos(2), l_hd_pos(2)], [l_fr_pos(3), l_hd_pos(3)], 'k-'); % connect forearm and hand
%     draw_segment(l_hd_quat, l_hd_pos); % orientation of hand
%     hold off; % enable refreshing
    draw_segment(l_up_quat_rviz, l_up_pos); % orientation of upper arm
    plot3([l_up_pos(1), l_fr_pos(1)], [l_up_pos(2), l_fr_pos(2)], [l_up_pos(3), l_fr_pos(3)], 'k-'); % connect upper arm and forearm  
    draw_segment(l_fr_quat_rviz, l_fr_pos); % orientation of forearm
    plot3([l_fr_pos(1), l_hd_pos(1)], [l_fr_pos(2), l_hd_pos(2)], [l_fr_pos(3), l_hd_pos(3)], 'k-'); % connect forearm and hand
    draw_segment(l_hd_quat_rviz, l_hd_pos); % orientation of hand
    hold off; % enable refreshing
    
    % store the frames for making gif later
    if record
        f = getframe;
        im = frame2im(f);
        [I, map] = rgb2ind(im, 256);
        % set Loopcount as Inf for the animation to play forever
        if i==1
            imwrite(I, map, gif_name, 'gif', 'Loopcount', Inf, 'DelayTime', 0.01);
        else
            imwrite(I, map, gif_name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.01);
        end
    end
    
    
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

%% Construct data array for wrist pos+ori and elbow pos
left_wrist_pos_ori_elbow_pos_path_1 = zeros(floor(size(data, 1)/10), 15);
left_base = [-0.06, 0.235, 0.395]; % the pos of the robot's shoulder in the RViz world
for i = 1 : floor(size(data, 1)/10)
    rotm_wrist = quat2rotm(left_hand.quat(i*10, :));
    % Since the position of the world frame is not the same as that of RViz(UR5) world, the position of left hand and left elbow should be converted with offset.   
    left_wrist_pos_ori_elbow_pos_path_1(i, 1:3) = left_hand.pos(i*10, :) - left_upperarm.pos(i*10, :) + left_base;
    left_wrist_pos_ori_elbow_pos_path_1(i, 4:12) = reshape(rotm_wrist', [1,9]);
    left_wrist_pos_ori_elbow_pos_path_1(i, 13:15) = left_forearm.pos(i*10, :) - left_upperarm.pos(i*10, :) + left_base;
end


%% Write data to .h5
% h5create('mocap_paths.h5', 'left_upperarm_path_1', size([left_upperarm.pos, left_upperarm.quat])); % create before writing
% h5create('mocap_paths.h5', 'left_forearm_path_1', size([left_forearm.pos, left_forearm.quat]));
% h5create('mocap_paths.h5', 'left_hand_path_1', size([left_hand.pos, left_hand.quat]));
% h5write('mocap_paths.h5', 'left_upperarm_path_1', [left_upperarm.pos, left_upperarm.quat]);
% h5write('mocap_paths.h5', 'left_forearm_path_1', [left_forearm.pos, left_forearm.quat]);
% h5write('mocap_paths.h5', 'left_hand_path_1', [left_hand.pos, left_hand.quat]);
% h5create('mocap_paths.h5', '/left_up_fr_hd_rot_path_1', size(left_up_fr_hd_rot_path_1'));
% h5write('mocap_paths.h5', '/left_up_fr_hd_rot_path_1', left_up_fr_hd_rot_path_1');
h5create('mocap_wrist_pos_ori_elbow_pos_paths.h5', '/left_wrist_pos_ori_elbow_pos_wave_hand_motion', size(left_wrist_pos_ori_elbow_pos_path_1'));
h5write('mocap_wrist_pos_ori_elbow_pos_paths.h5', '/left_wrist_pos_ori_elbow_pos_wave_hand_motion', left_wrist_pos_ori_elbow_pos_path_1');