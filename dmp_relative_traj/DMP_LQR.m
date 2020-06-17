function model = DMP_LQR(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, display)
% Enhanced dynamic movement primitive (DMP) model trained with EM by using a Gaussian mixture 
% model (GMM) representation, with full covariance matrices coordinating the different variables 
% in the feature space, and by using the task-parameterized model formalism. After learning 
% (i.e., autonomous organization of the basis functions (position and spread), Gaussian mixture 
% regression (GMR) is used to regenerate the path of a spring-damper system, resulting in a 
% nonlinear force profile. The gains of the spring-damper system are further refined by LQR 
% based on the retrieved covariance information.  
%
% If this code is useful for your research, please cite the related publication:
% @incollection{Calinon19chapter,
% 	author="Calinon, S. and Lee, D.",
% 	title="Learning Control",
% 	booktitle="Humanoid Robotics: a Reference",
% 	publisher="Springer",
% 	editor="Vadakkepat, P. and Goswami, A.", 
% 	year="2019",
% 	doi="10.1007/978-94-007-7194-9_68-1",
% 	pages="1--52"
% }
% 
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
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
model.nbStates = nbStates; %5; %Number of states in the GMM
model.nbVar = nbVarPos + 1; %3; %Number of variables [s,F1,F2] (decay term and perturbing force)
model.nbFrames = 1; %Number of candidate frames of reference (centered on goal position)
model.kP = kP; %50; %Stiffness gain
model.kV = kV; %(2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = alpha; %1.0; %Decay factor
model.dt = 0.01; %Duration of time step
model.nbVarPos = nbVarPos; %model.nbVar-1; %Dimension of spatial variables
model.rFactor = 5E-5; %Weighting term for the minimization of control commands in LQR
% nbData = 200; %Number of datapoints in a trajectory
% nbSamples = 4; %Number of demonstrations

%Canonical system parameters
A = kron([0 1; 0 0], eye(model.nbVarPos)); 
B = kron([0; 1], eye(model.nbVarPos)); 
C = kron([1,0],eye(model.nbVarPos));
%Discretize system (Euler method)
Ad = A*model.dt + eye(size(A));
Bd = B*model.dt;
%Control cost matrix
R = eye(model.nbVar) * model.rFactor;
R = kron(eye(nbData),R);

%Create transformation matrix to compute xhat = x + dx*kV/kP + ddx/kP
K1d = [1, model.kV/model.kP, 1/model.kP];
K = kron(K1d,eye(model.nbVarPos));


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demos=[];
% load('data/2Dletters/G.mat');
% sIn(1) = 1; %Initialization of decay term
% for t=2:nbData
% 	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
% end
sIn = exp(-model.alpha * (0:nbData-1)*model.dt);


DataDMP = zeros(model.nbVar,1,nbData*nbSamples);
Data=[];
for n=1:nbSamples
	%Task parameters (canonical coordinate system centered on the end-trajectory target)
	s(n).p(1).A = eye(model.nbVar);
	s(n).p(1).b = [0; traj_dataset{n}(end, :)'];
	%Demonstration data as [x;dx;ddx]
	s(n).Data = spline(1:size(traj_dataset{n}, 1), traj_dataset{n}', linspace(1, size(traj_dataset{n}, 1), nbData)); %Resampling
	Data = [Data s(n).Data]; %Original data
	s(n).Data = [s(n).Data; gradient(s(n).Data)/model.dt]; %Velocity computation
	s(n).Data = [s(n).Data; gradient(s(n).Data(end-model.nbVarPos+1:end,:))/model.dt]; %Acceleration computation
	s(n).Data = [sIn; K*s(n).Data]; %xhat computation
	s(n).Data = s(n).p(1).A \ (s(n).Data - repmat(s(n).p(1).b,1,nbData)); %Observation from the perspective of the frame
	DataDMP(:,1,(n-1)*nbData+1:n*nbData) = s(n).Data; %Training data as [s;xhat]
end


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_tensorGMM_kmeans(DataDMP, model); 
model = init_tensorGMM_timeBased(DataDMP, model); 
model = EM_tensorGMM(DataDMP, model);


%% Nonlinear force and motion retrieval
% set new goal and new start 
xTar = traj_dataset{1}(end, :)';% + [0, -0.5, 1]';
xStart = traj_dataset{1}(1, :)';% + [0, 0.5, 1]'; %Offset added to test generalization capability

%Task-adaptive spring-damper attractor path retrieval
r(n).p(1).A = eye(model.nbVar);
r(n).p(1).b = [0; xTar];  
[r(n).Mu, r(n).Sigma] = productTPGMM0(model, r(n).p); 
r(n).Priors = model.Priors;
r(n).nbStates = model.nbStates;
[r(1).currTar, r(1).currSigma] = GMR(r(n), sIn, 1, 2:model.nbVar); 

%LQR tracking (discrete version)
Q = zeros(model.nbVarPos*2, model.nbVarPos*2);
R = eye(model.nbVarPos) * model.rFactor;
P = zeros(model.nbVarPos*2, model.nbVarPos*2, nbData);
P(1:model.nbVarPos,1:model.nbVarPos,end) = inv(r(1).currSigma(:,:,nbData));
for t=nbData-1:-1:1
	Q(1:model.nbVarPos,1:model.nbVarPos) = inv(r(1).currSigma(:,:,t));
	P(:,:,t) = Q - Ad' * (P(:,:,t+1) * Bd / (Bd' * P(:,:,t+1) * Bd + R) * Bd' * P(:,:,t+1) - P(:,:,t+1)) * Ad;
end
%Motion retrieval 
x = xStart; %Data(1:model.nbVarPos,1);
dx = zeros(model.nbVarPos,1);
for t=1:nbData
	K = (Bd' * P(:,:,t) * Bd + R) \ Bd' * P(:,:,t) * Ad;
	ddx = K * ([r(1).currTar(:,t); zeros(model.nbVarPos,1)] - [x; dx]);
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	repro(:,t) = x;
end	

%% Display results
if display
    figure;
    plot3(Data(1, :), Data(2, :), Data(3, :), 'b.'); hold on; grid on;
    plot3(repro(1, :), repro(2, :), repro(3, :), 'r*');
    title('Reproduced trajectory and the original trajectory');
    %view(0, 90);
    xlabel('x'); ylabel('y'); zlabel('z');
    
    
    % figure;
    % sgtitle('Reproduced and original nonlinear force profile');
    % subplot(3, 1, 1); plot(DataDMP(1+1, :), 'b-'); hold on; grid on;
    % plot(Yr(:, 1), 'r-');
    % ylabel('X Force'); title('x');
    % subplot(3, 1, 2); plot(DataDMP(2+1, :), 'b-'); hold on; grid on;
    % plot(Yr(:, 2), 'r-');
    % ylabel('Y Force'); title('y');
    % subplot(3, 1, 3); plot(DataDMP(3+1, :), 'b-'); hold on; grid on;
    % plot(Yr(:, 3), 'r-');
    % ylabel('Z Force'); title('z');
    
    
    figure;
    sgtitle('Reproduced and original trajectory - split');
    subplot(3, 1, 1); plot(Data(1, :), 'b-'); hold on; grid on;
    plot(repro(1, :), 'r-');
    ylabel('X motion'); title('x');
    subplot(3, 1, 2); plot(Data(2, :), 'b-'); hold on; grid on;
    plot(repro(2, :), 'r-');
    ylabel('Y motion'); title('y');
    subplot(3, 1, 3); plot(Data(3, :), 'b-'); hold on; grid on;
    plot(repro(3, :), 'r-');
    ylabel('Z motion'); title('z');
    
end


end