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

%{
N = 526;
eul_angle = quat2eul(left_hand.quat(N, 1:4)) % quat2eul in MATLAB uses quaternion in (w,x,y,z) form!!!    
%}

%% Post-processing
% delete empty frame


% shift coordinate frame
%{
init_quat = [0, 0, 0, -1.0];%left_hand.quat(1,:);
init_eul = quat2eul(init_quat(1:3)) % the initial euler angles

quat_shift = eul2quat([0, -pi/2, -pi/2], 'ZYX'); % (w,x,y,z)
quatmultiply()
%}


% smoothing



%% Visualization
figure;
grid on;
quat_ori = [1, 0, 0, 0]; % [w,x,y,z]
pos_ori = [0, 0, 0];

for i = 1:round(size(f, 1)/10)

    % prepare parameters
    quat_shift = eul2quat([pi, -pi/2, -pi/2], 'ZYZ'); % (w,x,y,z)
    l_up_quat = quatmultiply(quat_shift, left_upperarm.quat(i*10, :));
    l_fr_quat = quatmultiply(quat_shift, left_forearm.quat(i*10, :));
    l_hd_quat = quatmultiply(quat_shift, left_hand.quat(i*10, :)); 
    l_up_pos = (quat2rotm(quat_shift)*left_upperarm.pos(i*10, :)')';
    l_fr_pos = (quat2rotm(quat_shift)*left_forearm.pos(i*10, :)')';
    l_hd_pos = (quat2rotm(quat_shift)*left_hand.pos(i*10, :)')';

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


