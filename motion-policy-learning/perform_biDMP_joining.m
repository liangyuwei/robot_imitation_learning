function [r_l, r_r] = perform_biDMP_joining(DMP_name_l, DMP_name_r, nbStates, nbVar, nbVarPos, kP_l, kV_l, kP_r, kV_r, alpha, dt, new_goal_l, new_start_l, new_goal_r, new_start_r, exp_rotm_l, exp_rotm_r)
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
% indices to select DMP
id_DMP_l = 1;
id_DMP_r = 1;
% load the params from recorded mat file
load learned_DMP_params.mat
for j = 1 : length(DMP_params)
    if isequal(DMP_params(j).name, DMP_name_l{id_DMP_l})
        model_l.Sigma = DMP_params(j).Sigma;
        model_l.Mu = DMP_params(j).Mu;
        MuF_l = DMP_params(j).Weights;
    elseif isequal(DMP_params(j).name, DMP_name_r{id_DMP_r})
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
x_l = new_start_l(:, id_DMP_l); xTar_l = new_goal_l(:, id_DMP_l);
x_r = new_start_r(:, id_DMP_r); xTar_r = new_goal_r(:, id_DMP_r);
% set initial velocity
dx_l = zeros(model_l.nbVarPos,1); dx_r = zeros(model_r.nbVarPos,1);
% set initial activation 
H_l = zeros(model_l.nbStates, 1);
H_r = zeros(model_r.nbStates, 1);
% store the result
r_l.Data = [];
r_r.Data = [];
% start iteration to generate new path
threshold = 0.005;
while (sIn_tmp_l >= threshold || sIn_tmp_r >= threshold) % wait until both DMP sequences reach the end 
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
    if(sIn_tmp_l >= threshold) % record the result only if the sequence has not reached the end
    	r_l.Data = [r_l.Data, x_l]; 
    else
        % it's better to keep recording even after it has ended... just so the paths of two arms are aligned, good for visualization    
        r_l.Data = [r_l.Data, r_l.Data(:, end)];
    end
    
    ddx_r = L_r * [xTar_r - x_r; -dx_r] + currF_r * sIn_tmp_r; 
	dx_r = dx_r + ddx_r * model_r.dt;
	x_r = x_r + dx_r * model_r.dt;
    if(sIn_tmp_r >= threshold) % record the result only if the sequence has not reached the end
        r_r.Data = [r_r.Data, x_r];
    else
        % it's better to keep recording even after it has ended... just so the paths of two arms are aligned, good for visualization
        r_r.Data = [r_r.Data, r_r.Data(:, end)];
    end
    
    % compute difference between two eef's poses
    rotm_l = eul2rotm([r_l.Data(6, end), r_l.Data(5, end), r_l.Data(4, end)]);
    rotm_r = eul2rotm([r_r.Data(6, end), r_r.Data(5, end), r_r.Data(4, end)]);
    rel_rotm_l = rotm_l' * exp_rotm_l; 
    rel_rotm_r = rotm_r' * exp_rotm_r;
    if(id_DMP_l == 1)
        dist_l = norm(rotm2eul(rel_rotm_l)) * 30; % lag of the left arm
    else
        dist_l = 0; % should disable constraint checking for the last DMP...
        % if more DMPs are concatenated, constraint switching should be added and exp_rel_rotm should be a cell   
    end
    if(id_DMP_r == 1)
        dist_r = norm(rotm2eul(rel_rotm_r)) * 30;
    else
        dist_r = 0;
    end
    
    % record timestamps and set the next time stamp and add coupling terms!!!
    sIn_l = [sIn_l, sIn_tmp_l];
    sIn_r = [sIn_r, sIn_tmp_r];
    percent = 0.95; % start constraint checking after finishing 95% of the whole path
    slow_threshold = 1-percent; 
    if(sIn_tmp_l > slow_threshold) %(1+percent)*threshold) 
        sIn_tmp_l = sIn_tmp_l - 2 * model_l.alpha * sIn_tmp_l * model_l.dt; %%%%% make the clock of the left arm faster!!! %%%%    
    else
        sIn_tmp_l = sIn_tmp_l - 2 * model_l.alpha * sIn_tmp_l * model_l.dt / (1+dist_r); % add coupling when approaching threshold       
    end
    % The problem resides in the same time evolution induced by using two canonical systems with the same settings(parameter alpha)  
    % as well as the same dt(discretization)
    % model_l.alpha * sIn_tmp_l * model_l.dt; %% try to modify the imitation traj instead of changing the internal clock;..... useless... the problem is not here %%
    %   %2 * model_l.alpha * sIn_tmp_l * model_l.dt; %%%%% make the clock of the left arm faster!!! %%%%
    if(sIn_tmp_r > slow_threshold) %(1+percent)*threshold) 
        sIn_tmp_r = sIn_tmp_r - model_r.alpha * sIn_tmp_r * model_r.dt; 
    else
        sIn_tmp_r = sIn_tmp_r - model_r.alpha * sIn_tmp_r * model_r.dt / (1+dist_l); % add coupling when approaching threshold         
    end
    
    % switch to next DMP, for left arm
    if(sIn_tmp_l<threshold) % if the left arm's DMP reached the end        
        if(id_DMP_l < length(DMP_name_l)) % if there is a successor waiting
            id_DMP_l = id_DMP_l + 1;
            % assign parameters of the next DMP
            for j = 1 : length(DMP_params)
                if isequal(DMP_params(j).name, DMP_name_l{id_DMP_l})
                    model_l.Sigma = DMP_params(j).Sigma;
                    model_l.Mu = DMP_params(j).Mu;
                    MuF_l = DMP_params(j).Weights;
                end
            end
            % reset input
            sIn_tmp_l = 1;
            % set new start and goal
            x_l = new_start_l(:, id_DMP_l); xTar_l = new_goal_l(:, id_DMP_l);
            % reset the initial vel (maybe should be modified later)
            dx_l = zeros(model_l.nbVarPos,1); 
        end
    end
    
    % switch to next DMP, for right arm
    if(sIn_tmp_r < threshold) % if the left arm's DMP reached the end        
        if(id_DMP_r < length(DMP_name_r)) % if there is a successor waiting
            id_DMP_r = id_DMP_r + 1;
            % assign parameters of the next DMP
            for j = 1 : length(DMP_params)
                if isequal(DMP_params(j).name, DMP_name_r{id_DMP_r})
                    model_r.Sigma = DMP_params(j).Sigma;
                    model_r.Mu = DMP_params(j).Mu;
                    MuF_r = DMP_params(j).Weights;
                end
            end
            % reset input
            sIn_tmp_r = 1;
            % set new start and goal
            x_r = new_start_r(:, id_DMP_r); xTar_r = new_goal_r(:, id_DMP_r);
            % reset the initial value (maybe should be modified later)
            dx_r = zeros(model_r.nbVarPos,1); 
        end
    end
    
end

% for debug
% pause;

