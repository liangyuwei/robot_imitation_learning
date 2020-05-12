function model = shape_modulation_add_viapoint(model, x_via, h_via, y_via, sigma_via)
%% This function manipulates the shape modulation for integrating new via-points.
% note that here we don't add via-points for quaternion data, since it's more complicated to do so than just integrating them into elementary trajectory.  
% Input: x_via, h_via, y_via - 1-dim data

% preparation
num_datapoints = size(model.phi, 1);
phi_x_via = zeros(size(model.Mu_w)); % interpolate from the already computed .phi
for n = 1 : size(phi_x_via, 2)
    phi_x_via(n) = interp1(linspace(0, 1, num_datapoints), model.phi(:, n), x_via);
end

%  conditional distribution parameters
L = model.Sigma_w * phi_x_via / (sigma_via + phi_x_via'*model.Sigma_w*phi_x_via);
mu_w = model.Mu_w + L * (y_via - h_via - phi_x_via'*model.Mu_w);
sigma_w = model.Sigma_w - L * phi_x_via' * model.Sigma_w;

% update the shape modulation parameters
model.Mu_w = mu_w;
model.Sigma_w = sigma_w;


end