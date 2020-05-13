function model = shape_modulation_add_viapoint(model, x_via, h_via, y_via, sigma_via, time_range)
%% This function manipulates the shape modulation for integrating new via-points.
% note that here we don't add via-points for quaternion data, since it's more complicated to do so than just integrating them into elementary trajectory.  
% Input: x_via, h_via, y_via - 1-dim data

% preparation
t = time_range;  %Time range
phi_x_via = zeros(size(model.Mu_w)); % interpolate from the already computed .phi
tMu = linspace(t(1), t(end), size(model.Mu_w, 1));
for n = 1 : size(phi_x_via, 1)
    % compute phi(x_via) directly
    phi_x_via(n) = gaussPDF(x_via, tMu(n), 1E-2);
end

%  conditional distribution parameters
L = model.Sigma_w * phi_x_via / (sigma_via + phi_x_via'*model.Sigma_w*phi_x_via);
% tmp = model.Sigma_w * phi_x_via / (sigma_via + phi_x_via'*model.Sigma_w*phi_x_via) * phi_x_via'
% mu_w = model.Mu_w + L * (y_via - h_via - phi_x_via'*model.Mu_w);
% sigma_w = model.Sigma_w - L * phi_x_via' * model.Sigma_w;
mu_w = model.Mu_w + model.Sigma_w * phi_x_via / (sigma_via + phi_x_via'*model.Sigma_w*phi_x_via) * (y_via - h_via - phi_x_via'*model.Mu_w);
sigma_w = model.Sigma_w - (model.Sigma_w * phi_x_via * phi_x_via' * model.Sigma_w)/(sigma_via + phi_x_via'*model.Sigma_w*phi_x_via);


% update the shape modulation parameters
model.Mu_w = mu_w;
model.Sigma_w = sigma_w;


end