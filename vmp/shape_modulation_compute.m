function f_seq = shape_modulation_compute(model, time_range, sigma_y, quat_or_not)
%% This function only computes f(x) values.
% Input: model - should include Mu_w, Sigma_w and Psi for calculation.
%        model processes for 1-dim data.
% Output: f_seq - a sequence, should be of the size (1, num_datapoints) or (4, num_datapoints)
% %%%%%%%%%%%%%%%
% To ensure continuity, random parameter vecter w is fixed for a single
% trajectory, so this function outputs a whole trajectory.   

num_datapoints = size(time_range, 2);

% noise term
% sigma_y = 0.005;


%% Get w from probability distribution
if quat_or_not
    % quaternion data
    n_y = sqrt(sigma_y) .* randn(3, num_datapoints); %gaussian noise; for quaternion, better be 0.00001
    %% maybe we should just use Mu_w ??? // I have commented out the eye(.) part in Sigma_w calculation in proMP_get_mu_sigma.m file
    found = false;
    count = 0;
    while ~found
        w = mvnrnd(model(1).Mu_w, model(1).Sigma_w)';
        tmp_f = (model(1).Psi * w)'; %model(1).Mu_w)';
        qx = tmp_f(1:3:end); qy = tmp_f(2:3:end); qz = tmp_f(3:3:end);
        f_seq = [qx; qy; qz] + n_y;
        found = isempty(find(sum(f_seq .^2)>1, 1)); % check if the result is feasible
        count = count + 1;
        if count > 100
            disp('Calculating shape modulation for quaternion data: Something might be wrong, a feasible result hasn''t been found for over 100 iterations...');
            break;
        end
    end
    % compute the real part of quaternion
    qw = sqrt(1-sum(f_seq .^2));
    f_seq = [qw; f_seq];
%     plot(1:num_datapoints, qw, 'b.'); % check continuity
else
    % pos or angle data
    n_y = sqrt(sigma_y) .* randn(1, num_datapoints); % gaussian noise
    % to ensure continuity, w is identical for one single trajectory
    w = mvnrnd(model.Mu_w, model.Sigma_w)';
    f_seq = (model.Psi * w)' + n_y;
    % .Psi is of the same size of time_range(set during learning VMP). so no need to recompute again.
    % In addition, .Psi is already the transpose, just use it directly.
end






end