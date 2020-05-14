function y_seq = get_final(f_seq, h_seq, pos_and_glove_id, quat_id)
%% This function obtains the final trajectory: y_seq = f_seq + h_seq for pos/angle data, y_seq = h_seq * f_seq for quaternion data.

%% Initialization
y_seq = zeros(size(h_seq));


%% For pos/angle data
y_seq(pos_and_glove_id, :) = h_seq(pos_and_glove_id, :) + f_seq(pos_and_glove_id, :);


%% For quaternion data
for q_id = 1:2
    dof_id = quat_id((q_id-1)*4+1 : q_id*4);
    for t = 1:size(h_seq, 2)
        f = y_seq(dof_id, t)';
        h = h_seq(dof_id, t)';
        % SLERP interpolation
        y_seq(dof_id, t) = quatmultiply(h, f)';
    end
end


end