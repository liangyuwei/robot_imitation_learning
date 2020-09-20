function cost = human_ik_cost(x)
%% This function is used for Human IK optimization.

%% Obtain related data
global shoulder_pos upperarm_length forearm_length left_or_right
global elbow_pos_goal wrist_pos_goal wrist_rot_goal


%% Perform FK
[elbow_pos, wrist_pos, wrist_rot] = human_fk(shoulder_pos, x, upperarm_length, forearm_length, left_or_right);


%% Calculate the cost
% coefficients
K_wrist_pos = 5.0; 
K_wrist_rot = 1.0;
K_elbow_pos = 1.0;
% elbow position
elbow_pos_cost = norm(elbow_pos - elbow_pos_goal);
% wrist position
wrist_pos_cost = norm(wrist_pos - wrist_pos_goal);
% wrist orientation
wrist_ori_cost = acos((trace(wrist_rot'*wrist_rot_goal)-1)/2);
% in total
cost = K_wrist_pos * wrist_pos_cost + K_wrist_rot * wrist_ori_cost + K_elbow_pos * elbow_pos_cost;


end