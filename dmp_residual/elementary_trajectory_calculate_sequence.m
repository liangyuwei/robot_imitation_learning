function h_seq = elementary_trajectory_calculate_sequence(ele_traj_struct_all, time_range, pos_and_glove_id, quat_id)
%% Computes the h(x) sequence

%% Initialize
DOF = size(pos_and_glove_id, 2) + size(quat_id, 2);
h_seq = zeros(DOF, size(time_range, 2));

%% For pos/angle data
for pg_id = 1 : size(pos_and_glove_id, 2)
    dof_id = pos_and_glove_id(pg_id);
    for n = 1 : size(time_range, 2)
        h_seq(dof_id, n) = elementary_trajectory_calculate(ele_traj_struct_all{pg_id}, time_range(n), false);
    end
end

%% For quaternion data
for q_id = 1:2
    dof_id = quat_id((q_id-1)*4+1 : q_id*4);
    for n = 1 : size(time_range, 2)
        h_seq(dof_id, n) = elementary_trajectory_calculate(ele_traj_struct_all{size(pos_and_glove_id, 2)+q_id}, time_range(n), true);
    end
end


end