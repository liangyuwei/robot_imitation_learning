function [elbow_pos, wrist_pos, wrist_rot] = human_fk(p_s, q, l_up, l_fr, left_or_right)
%% This function computes Forward Kinematics on a human arm.
% Input:
%   p_s - shoulder position in world frame, should be of size 3 x 1
%   q - human joint angle (the same as WuKong Humanoid)
%   l_up - length of upperarm link(determined by mocap data, may change under different joint state due to offset of marker)
%   l_fr - length of forearm link
%   left_or_right - compute FK for left arm or right arm (left arm model and local frames are rotated around z-axis by 180 degree)

%% Prep
UX = [1, 0, 0];
UY = [0, 1, 0];
UZ = [0, 0, 1];

%% Calculate relative transforms
if (left_or_right)
    T0 = [eul2rotm(UZ*pi, 'XYZ'), p_s; ...
          0, 0, 0, 1]; % homogeneous transform of shoulder under world frame    
else
    T0 = [eye(3), p_s; ...
          0, 0, 0, 1]; % homogeneous transform of shoulder under world frame    
end

T1 = [axang2rotm([UY, q(1)]), [0, 0, 0]'; ...
      0, 0, 0, 1]; % for q1

T2 = [axang2rotm([UX, q(2)]), [0, 0, 0]'; ...
      0, 0, 0, 1];
  
T3 = [axang2rotm([UZ, q(3)]), [0, 0, 0]'; ...
      0, 0, 0, 1];
  
T4 = [axang2rotm([UX, q(4)]), [0, 0, -l_up]'; ...
      0, 0, 0, 1];
  
T5 = [axang2rotm([UZ, q(5)]), [0, 0, 0]'; ...
      0, 0, 0, 1];
  
T6 = [axang2rotm([UX, q(6)]), [0, 0, -l_fr]'; ...
      0, 0, 0, 1];
  
T7 = [axang2rotm([UY, q(7)]), [0, 0, 0]'; ...
      0, 0, 0, 1];

%% Get elbow positions and wrist pose
elbow_tform = T0 * T1 * T2 * T3 * T4 * T5; % relative transform, post-multiply !!!
wrist_tform = T0 * T1 * T2 * T3 * T4 * T5 * T6 * T7;

elbow_pos = elbow_tform(1:3, end);
wrist_pos = wrist_tform(1:3, end);
wrist_rot = wrist_tform(1:3, 1:3); % should be transformed to local frame!!!   

% wrist orientation under q=zeros(7, 1) is different from the world frame, yet the rotation matrix of T7 is eye(3)     
% therefore, there should be a conversion, so as to utilize the orientation data of wrist(from mocap where eye(3) is exactly the same as world frame)    
world_R_local = [1, 0, 0; ...
                 0, -1, 0; ...
                 0, 0, -1];       
world_R_wrist = world_R_local * wrist_rot; % local_R_wrist(current result) = local_R_world * world_R_wrist(desired output for us to compare with the goal from mocap data)
wrist_rot = world_R_wrist; % assign the adjusted orientation


end