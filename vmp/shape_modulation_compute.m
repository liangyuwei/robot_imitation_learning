function f_x = shape_modulation_compute(model, time_range, sigma_y, quat_or_not)
%% This function only computes f(x) values.
% Input: model - should include Mu_w, Sigma_w and Psi for calculation.
%        model processes for 1-dim data.
% Output: f_x - should be of the size (1, num_datapoints) or (4, num_datapoints)
% %%%%%%%%%%%%%%%
% To ensure continuity, random parameter vecter w is fixed for a single
% trajectory, so this function outputs a whole trajectory.   

num_datapoints = size(time_range, 2);

% noise term
sigma_y = 0.005;
n_y = sqrt(sigma_y) .* randn(size(time_range));


%% Get w from probability distribution
if quat_or_not
    % quaternion data
    
else
    % pos or angle data
    % to ensure continuity, w is identical for one single trajectory
    w = mvnrnd(model.Mu_w, model.Sigma_w)';
    f_x = (model.Psi * w)' + n_y;
    % .Psi is of the same size of time_range(set during learning VMP). so no need to recompute again.
    % In addition, .Psi is already the transpose, just use it directly.
end






end