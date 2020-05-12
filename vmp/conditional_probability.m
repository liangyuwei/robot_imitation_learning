function cond_prob = conditional_probability(model, ele_traj_struct, x_via, y_via, sigma_via)
%% Compute conditional probability at the given point
% Input: model - the current shape modulation model;
%        x_via, y_via - canonical time and value of the given via-point
%        sigma_via - uncertainty of the via-point. 0 indicates that it must be gone through    
% %%%%%%%%
% Input 1-D data, only compute conditional probability for pos or angle data. 
% Quaternion via-points are directly integrated into elementary trajectory.    

    h_via = elementary_trajectory_compute(ele_traj_struct, x_via, false); % pos or angle data
    cond_prob = gaussPDF(y_via, h_via + model.Psi * model.Mu_w, model.Psi * model.Sigma_w * model.Psi' + sigma_via);


end





