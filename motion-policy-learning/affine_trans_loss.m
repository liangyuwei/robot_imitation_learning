function loss = affine_trans_loss(M)
%% This is the loss function for the optimization of the affine transform's parameter.

global tmp_Q % global variables need to be declared before being used.
N = size(tmp_Q, 2);

[elbow_traj, wrist_traj] = FK_panda_arm(tmp_Q); % don't forget to use the transpose as the true M!!!
% elbow_traj/wrist_traj should be 3 x (length - t_start_trans + 1)

[elbow_traj_affine, wrist_traj_affine] = FK_panda_arm( repmat(tmp_Q(:, 1), 1, N) ...
                                                       + M' * (tmp_Q(:, :) - repmat(tmp_Q(:, 1), 1, N)) );

loss = sum(sum((elbow_traj - elbow_traj_affine) .^ 2)) + sum(sum((wrist_traj - wrist_traj_affine) .^ 2));

end