function [r_l, r_r] = perform_biDMP(DMP_name_l, DMP_name_r, nbStates, nbVar, nbVarPos, kP_l, kV_l, kP_r, kV_r, alpha, dt, new_goal_l, new_start_l, new_goal_r, new_start_r)
% Dynamical movement primitive (DMP) with locally weighted regression (LWR). 
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon16JIST,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
% }
% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

% addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model_l.nbStates = nbStates; %5; %Number of activation functions (i.e., number of states in the GMM)
model_l.nbVar = nbVar; % 1; %Number of variables for the radial basis functions [s] (decay term)
model_l.nbVarPos = nbVarPos; %2; %Number of motion variables [x1,x2] 
model_l.kP = kP_l; % 50; %Stiffness gain
model_l.kV = kV_l; % (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model_l.alpha = alpha; %1.0; %Decay factor
model_l.dt = dt; %0.01; %Duration of time step
L_l = [eye(model_l.nbVarPos)*model_l.kP, eye(model_l.nbVarPos)*model_l.kV]; %Feedback term
model_r.nbStates = nbStates; %5; %Number of activation functions (i.e., number of states in the GMM)
model_r.nbVar = nbVar; % 1; %Number of variables for the radial basis functions [s] (decay term)
model_r.nbVarPos = nbVarPos; %2; %Number of motion variables [x1,x2] 
model_r.kP = kP_r; % 50; %Stiffness gain
model_r.kV = kV_r; % (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model_r.alpha = alpha; %1.0; %Decay factor
model_r.dt = dt; %0.01; %Duration of time step
L_r = [eye(model_r.nbVarPos)*model_r.kP, eye(model_r.nbVarPos)*model_r.kV]; %Feedback term


%% Pre-setup
% sIn(1) = 1; %Initialization of decay term
% for t=2:nbData
% 	sIn(t) = sIn(t-1) - model_l.alpha * sIn(t-1) * model_l.dt; %Update of decay term (ds/dt=-alpha s)
% end


%% Load the learned parameters for generating new trajectory
% load the params from recorded mat file
load learned_DMP_params.mat
for j = 1 : length(DMP_params)
    if isequal(DMP_params(j).name, DMP_name_l)
        model_l.Sigma = DMP_params(j).Sigma;
        model_l.Mu = DMP_params(j).Mu;
        MuF_l = DMP_params(j).Weights;
    elseif isequal(DMP_params(j).name, DMP_name_r)
        model_r.Sigma = DMP_params(j).Sigma;
        model_r.Mu = DMP_params(j).Mu;
        MuF_r = DMP_params(j).Weights;
    end
end


%% Motion retrieval with DMP
% reset the clock
sIn_tmp_l = 1; sIn_l = [];
sIn_tmp_r = 1; sIn_r = [];
% set the start and goal
x_l = new_start_l; xTar_l = new_goal_l;
x_r = new_start_r; xTar_r = new_goal_r;
% set initial velocity
dx_l = zeros(model_l.nbVarPos,1); dx_r = zeros(model_r.nbVarPos,1);
% set initial activation 
H_l = zeros(model_l.nbStates, 1);
H_r = zeros(model_r.nbStates, 1);
% store the result
r_l.Data = [];
r_r.Data = [];
% start iteration to generate new path
while sIn_tmp_l >= 0.005
    % update H, activation
    for i = 1:model_l.nbStates
        H_l(i,:) = gaussPDF(sIn_tmp_l, model_l.Mu(:,i), model_l.Sigma(:,:,i));
        H_r(i,:) = gaussPDF(sIn_tmp_r, model_r.Mu(:,i), model_r.Sigma(:,:,i));
    end
    H_l = H_l ./ repmat(sum(H_l),model_l.nbStates,1);
    H_r = H_r ./ repmat(sum(H_r),model_r.nbStates,1);
    currF_l = MuF_l * H_l; 
    currF_r = MuF_r * H_r; 

	%Compute acceleration, velocity and position	 
    ddx_l = L_l * [xTar_l - x_l; -dx_l] + currF_l * sIn_tmp_l; 
	dx_l = dx_l + ddx_l * model_l.dt;
	x_l = x_l + dx_l * model_l.dt;
	r_l.Data = [r_l.Data, x_l];
    
    ddx_r = L_r * [xTar_r - x_r; -dx_r] + currF_r * sIn_tmp_r; 
	dx_r = dx_r + ddx_r * model_r.dt;
	x_r = x_r + dx_r * model_r.dt;
	r_r.Data = [r_r.Data, x_r];    
    
    % record timestamps; 
    % set the next time stamp and add coupling terms!!!
    sIn_l = [sIn_l, sIn_tmp_l];
    sIn_r = [sIn_r, sIn_tmp_r];
    sIn_tmp_l = sIn_tmp_l - model_l.alpha * sIn_tmp_l * model_l.dt; 
    sIn_tmp_r = sIn_tmp_r - model_r.alpha * sIn_tmp_r * model_r.dt;     
    
end

% for debug
% pause;

