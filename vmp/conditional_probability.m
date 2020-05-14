function cond_prob = conditional_probability(model, ele_traj_struct, x_via, y_via, sigma_via, time_range)
%% Compute conditional probability at the given point
% Input: model - the current shape modulation model;
%        x_via, y_via - canonical time and value of the given via-point
%        sigma_via - uncertainty of the via-point. 0 indicates that it must be gone through    
% %%%%%%%%
% Input 1-D data, only compute conditional probability for pos or angle data. 
% Quaternion via-points are directly integrated into elementary trajectory.    

    % preparation
    % h(x_via)
%     h_via = elementary_trajectory_compute(ele_traj_struct, x_via, false); % pos or angle data
    h_via = elementary_trajectory_compute_linear(ele_traj_struct, x_via, false); % pos or angle data

    % phi(x_via)
    t = time_range;  %Time range
    phi_x_via = zeros(size(model.Mu_w)); % interpolate from the already computed .phi
    tMu = linspace(t(1), t(end), size(model.Mu_w, 1));
    for n = 1 : size(phi_x_via, 1)
        % compute phi(x_via) directly
        phi_x_via(n) = gaussPDF(x_via, tMu(n), 1E-2);
    end
    
    % compute conditional probability
    cond_prob = gaussPDF(y_via, h_via + phi_x_via' * model.Mu_w, phi_x_via' * model.Sigma_w * phi_x_via + sigma_via);

    if isnan(cond_prob)
        pause;
    end

end





