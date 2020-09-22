function cost = human_ik_cost(x, tracking_goal)
%% This function is used for Human IK optimization as the cost function.


%% Obtain goal information    
% global tracking_goal
shoulder_pos_goal = tracking_goal.shoulder_pos_goal;
elbow_pos_goal = tracking_goal.elbow_pos_goal;
wrist_pos_goal = tracking_goal.wrist_pos_goal;

shoulder_rot_goal = tracking_goal.shoulder_rot_goal;
elbow_rot_goal = tracking_goal.elbow_rot_goal;
wrist_rot_goal = tracking_goal.wrist_rot_goal;

upperarm_length = tracking_goal.upperarm_length;
forearm_length = tracking_goal.forearm_length;
left_or_right = tracking_goal.left_or_right;


%% Perform FK
[T_shoulder, T_elbow, T_wrist] = human_fk(shoulder_pos_goal, x, upperarm_length, forearm_length, left_or_right, false);
% position 
shoulder_pos = T_shoulder(1:3, end);
elbow_pos = T_elbow(1:3, end);
wrist_pos = T_wrist(1:3, end);
% orientation
shoulder_rot = T_shoulder(1:3, 1:3);
elbow_rot = T_elbow(1:3, 1:3);
wrist_rot = T_wrist(1:3, 1:3);


%% Calculate the cost
% coefficients (at least 7 constraints to solve for 7 joints)
K_shoulder_pos = 0.0; %1.0; % no need since shoulder position is set directly...   
K_elbow_pos = 10.0;%5.0; %1.0;
K_wrist_pos = 5.0; %1.0;
K_shoulder_rot = 0.0; %1.0;
K_elbow_rot = 0.0; %1.0;
K_wrist_rot = 1.0; 

% 1 - position costs
shoulder_pos_cost = norm(shoulder_pos - shoulder_pos_goal);
elbow_pos_cost = norm(elbow_pos - elbow_pos_goal);
wrist_pos_cost = norm(wrist_pos - wrist_pos_goal);

% 2 - orientation costs
shoulder_rot_cost = acos((trace(shoulder_rot'*shoulder_rot_goal)-1)/2);
elbow_rot_cost = acos((trace(elbow_rot'*elbow_rot_goal)-1)/2);
wrist_rot_cost = acos((trace(wrist_rot'*wrist_rot_goal)-1)/2);

% in total
cost = K_shoulder_pos * shoulder_pos_cost + K_elbow_pos * elbow_pos_cost + K_wrist_pos * wrist_pos_cost ...
     + K_shoulder_rot * shoulder_rot_cost + K_elbow_rot * elbow_rot_cost + K_wrist_rot * wrist_rot_cost;


end