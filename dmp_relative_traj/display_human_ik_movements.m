function display_human_ik_movements(q_l_traj, q_r_traj, shoulder_l_traj, shoulder_r_traj, len_up_l, len_up_r, len_fr_l, len_fr_r)
%% This function displays the human IK results.
% Input:
%   q_l_traj, q_r_traj - joint trajectories for left and right arms, should
%   be of size 7 x N
%   len_{up/fr/hd}_{l/r} - corresponding link lengths (could use average data)
%   shoulder_{l/r}_traj - shoulder positions

%% Prep
num_datapoints = size(q_l_traj, 2);
assert(num_datapoints == size(q_r_traj, 2), 'The length of left and right joint trajectories should be consistent!!!');


%% Perform FK to obtain transforms of local frames
T_shoulder_l_traj = zeros(4, 4, num_datapoints);
T_elbow_l_traj = zeros(4, 4, num_datapoints);
T_wrist_l_traj = zeros(4, 4, num_datapoints);
T_shoulder_r_traj = zeros(4, 4, num_datapoints);
T_elbow_r_traj = zeros(4, 4, num_datapoints);
T_wrist_r_traj = zeros(4, 4, num_datapoints);

for i = 1 : num_datapoints
    % left
    [T_shoulder_l_traj(:, :, i), T_elbow_l_traj(:, :, i), T_wrist_l_traj(:, :, i)] = human_fk(shoulder_l_traj(:, i), q_l_traj(:, i), len_up_l(i), len_fr_l(i), true, false);
    % right
    [T_shoulder_r_traj(:, :, i), T_elbow_r_traj(:, :, i), T_wrist_r_traj(:, :, i)] = human_fk(shoulder_r_traj(:, i), q_r_traj(:, i), len_up_r(i), len_fr_r(i), false, false);
end


%% 2 - transform to local frames of mocap markers         
figure;
for i = 1 : num_datapoints
    % left
    plot_local_frames(T_shoulder_l_traj(1:3, end, i), T_shoulder_l_traj(1:3, 1:3, i));
    plot_local_frames(T_elbow_l_traj(1:3, end, i), T_elbow_l_traj(1:3, 1:3, i));
    plot_local_frames(T_wrist_l_traj(1:3, end, i), T_wrist_l_traj(1:3, 1:3, i));    
    % right
    plot_local_frames(T_shoulder_r_traj(1:3, end, i), T_shoulder_r_traj(1:3, 1:3, i));
    plot_local_frames(T_elbow_r_traj(1:3, end, i), T_elbow_r_traj(1:3, 1:3, i));
    plot_local_frames(T_wrist_r_traj(1:3, end, i), T_wrist_r_traj(1:3, 1:3, i));    
    % info
    title('Human Arms Movement');
    xlabel('x'); ylabel('y'); zlabel('z');
    view(120, 30);
    axis equal % set suitable horizon for view
    hold off;
    pause(0.1);
    drawnow;
end




end