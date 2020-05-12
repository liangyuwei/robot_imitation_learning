function m = proMP_get_mu_sigma(nbStates, nbVar, nbSamples, nbData, traj_samples, time_range)
%% This function estimates the prior probability distribution of w from data.
% Code Adjusted from PbDlib, demo_proMP01.m.
% input: nbStates - Number of basis functions
%        nbVar - Dimension of position data (here: x1,x2)
%        nbSamples - Number of demonstrations
%        nbData - Number of datapoints in a trajectory
%        traj_samples - All demonstration samples, should be of the size (DOF, nbData, nbSamples)
%        time_range - Canonical time, internal clock

% find support functions
addpath('./m_fcts/');

% Parameters
% nbVar = 3; %Dimension of position data (here: x1,x2)
% nbSamples = num_imitation_data; %Number of demonstrations
% nbData = num_resampled_points; %Number of datapoints in a trajectory

% prepare data
x=[];
for n=1:nbSamples
	s(n).x = spline(1:size(traj_samples(:, :, n), 2), traj_samples(:, :, n), linspace(1,size(traj_samples(:, :, n), 2), nbData)); %Resampling
	x = [x, s(n).x(:)]; %Reorganize as trajectory datapoints 
end
t = time_range; %linspace(0,1,nbData); %Time range

% ProMP with radial basis functions 
%Compute basis functions Psi and activation weights w
tMu = linspace(t(1), t(end), nbStates);
m.phi = zeros(nbData,nbStates);
for i=1:nbStates
	m.phi(:,i) = gaussPDF(t, tMu(i), 1E-2);
% 	m(1).phi(:,i) = mvnpdf(t', tMu(i), 1E-2); 
end
% m(1).phi = m(1).phi ./ repmat(sum(m(1).phi,2),1,nbStates); %Optional rescaling
m.Psi = kron(m(1).phi, eye(nbVar)); %Eq.(27)
m.w = (m.Psi' * m.Psi + eye(nbVar*nbStates).*1E-8) \ m.Psi' * x; %m(1).w = pinv(m(1).Psi) * x'; %Eq.(28)
%Distribution in parameter space
m.Mu_w = mean(m.w,2);
m.Sigma_w = cov(m.w') ;%+ eye(nbVar*nbStates) * 1E0;  % only the diagonal blocks are non-zero, checked by sum()
% %%%%% here the +eye(...) part is commented out by LYW, 2020/05/12, since the diagonal terms are too huge, leading to huge w generated through mvnrnd
% %%%%% If this doesn't work well, try uncommenting the above expression and use Mu_w for shape modulation instead(in shape_modulation_compute.m)    


%Trajectory distribution
% m(1).Mu = m(1).Psi * m(1).Mu_w; %Eq.(29)
% m(1).Sigma = m(1).Psi * m(1).Sigma_w * m(1).Psi'; %Eq.(29)

% Sigma is too hugh due to error during imitation



end



