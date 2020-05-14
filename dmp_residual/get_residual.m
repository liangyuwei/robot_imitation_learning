function f_seq = get_residual(y_seq, h_seq, pos_and_glove_id, quat_id)
%% This function obtains the residual: f_seq = y_seq - h_seq for pos/angle data, f_seq = y_seq * inv(h_seq) for quaternion data.

%% Initialization
f_seq = zeros(size(h_seq));


%% For pos/angle data
f_seq(pos_and_glove_id, :) = y_seq(pos_and_glove_id, :) - h_seq(pos_and_glove_id, :);


%% For quaternion data
for q_id = 1:2
    dof_id = quat_id((q_id-1)*4+1 : q_id*4);
    for t = 1:size(h_seq, 2)
        y = y_seq(dof_id, t)';
        h = h_seq(dof_id, t)';
        % SLERP interpolation
        f_seq(dof_id, t) = quatmultiply(y, quatinv(h))';
    end
end


end