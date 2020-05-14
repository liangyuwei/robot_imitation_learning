function ele_traj_struct_all = elementary_trajectory_model(pass_points, pass_time, pos_and_glove_id, quat_id)
%% This function models a piece-wise linear function connecting all the via-points as well as start and goal.
% Input: pass_points - Include start, goal and all via-points, should be of the size (DOF * (#keypoints+2))   
%        imi_traj - imitation trajectory, y_seq, trajectory to be learned. be of the size (DOF * num_resampled_points)     
%        kp_list - keypoint id's, (1, #keypoints)
%        pass_time - x_via(time) at via-points, would be stored in .floor after modeling.    
% Output: the function coefficients, could be used for computing any points in-between.    

% DOF = size(pass_points, 1);
% num_turnpoints = size(pass_points, 2);

%% Initialize
ele_traj_struct_all = cell(3*4+14*2+2, 1); % 1 for each pos/angle dim, 1 for each quaternion data

%% For pos/angle data
for pg_id = 1 : size(pos_and_glove_id, 2)
    dof_id = pos_and_glove_id(pg_id);
    % set up start and goal
    ele_traj_struct = struct;
    ele_traj_struct.floor = [0, 1];
    ele_traj_struct.ab = zeros(2, 2); % a and b, the coefficients of the 1st-order linear function h(x) = a * x + b
    g = pass_points(dof_id, end); y0 = pass_points(dof_id, 1);
    ele_traj_struct.ab(1, :) = [g-y0, y0]; % initial: h(x) = (g-y0)*x + y0
    % update structure (insert via-points)
    for v = 1 : size(pass_time, 2)
        x_via = pass_time(v);
        h_via = pass_points(dof_id, v); % f_via == f_via at via-points
        ele_traj_struct = elementary_trajectory_insert_viapoint(ele_traj_struct, x_via, h_via, false);
    end
    % store the result
    ele_traj_struct_all{pg_id } = ele_traj_struct;
end

%% For quaternion data
for q_id = 1:2
    dof_id = quat_id((q_id-1)*4+1 : q_id*4);
    % set up start and goal
    ele_traj_struct = struct;
    ele_traj_struct.floor = [0, 1]; % 1 is added for locating via-points closer to the end (by find function)
    ele_traj_struct.quat = zeros(2,4);
    ele_traj_struct.quat(1, :) = pass_points(dof_id, 1)';
    ele_traj_struct.quat(2, :) = pass_points(dof_id, end)';  % row vector
    % update the structure (insert via-points)
    for v = 1 : size(pass_time, 2)
        x_via = pass_time(v);
        h_via = pass_points(dof_id, v)'; % f_via == f_via at via-points
        ele_traj_struct = elementary_trajectory_insert_viapoint(ele_traj_struct, x_via, h_via, true);
    end
    % store the result
    ele_traj_struct_all{size(pos_and_glove_id, 2)+q_id} = ele_traj_struct;
end



end