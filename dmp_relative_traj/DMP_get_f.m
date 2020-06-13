function DataDMP = DMP_get_f(traj_dataset, nbData, nbVarPos, kP, kV, alpha)
%% This function trains a DMP on the imitation trajectory
% addpath('./m_fcts/');
% input trajectory should be of size Length x DOF
% DMP is simply a point-attractive curve, plus a fixed curve (f), so it doesn't matter how f is modeled...
% DataDMP is the nonlinear force term, rad_basis * w / sum(rad_basis), with x excluded...
% %%%%%% nbData mush be large engough for initial velocity be 0, otherwise the reproduction result would be deviating !!! (during reproduction, initial velocity is set as 0) %%%%%%

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% num_datapoints = 50;
% model.nbStates = nbStates; %8; %5; %Number of activation functions (i.e., number of states in the GMM)
% model.nbVar = nbVar; %1; %Number of variables for the radial basis functions [s] (decay term)
model.nbVarPos = nbVarPos;%3; %2; %Number of motion variables [x1,x2]
model.kP = kP;%50; %Stiffness gain
model.kV = kV; %(2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = alpha; %1.0; %Decay factor
model.dt = 1/15; %1/nbData * 2; %Duration of time step
% nbData = num_datapoints; %200; %Length of each trajectory
% nbSamples = nbSamples; %1; %Number of demonstrations
L = [eye(model.nbVarPos)*model.kP, eye(model.nbVarPos)*model.kV]; %Feedback term


%% Load imitation data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
posId=[1:model.nbVarPos]; velId=[model.nbVarPos+1:2*model.nbVarPos]; accId=[2*model.nbVarPos+1:3*model.nbVarPos];
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
% decay term, wish it to be in consistent with the time_range on which pass_time is specified..
% sIn = linspace(1, 0, nbData);
% xTar = new_goal; %demos{1}.pos(:,end);
DataDMP=[];
%traj_dataset = ...; % {i} 1000 x 3
tmp = traj_dataset{1}';
xStart = tmp(:, 1);
xTar = tmp(:, end); % modified by LYW, column vector, 3 x 1
%Demonstration data as [x;dx;ddx]
s(1).Data = spline(1:size(traj_dataset{1}, 1), traj_dataset{1}', linspace(1, size(traj_dataset{1}, 1), nbData)); %Resampling
s(1).Data = [s(1).Data; gradient(s(1).Data)/model.dt]; %Velocity computation
s(1).Data = [s(1).Data; gradient(s(1).Data(end-model.nbVarPos+1:end,:))/model.dt]; %Acceleration computation
%Nonlinear forcing term
DataDMP = (s(1).Data(accId,:) - ...
    (repmat(xTar,1,nbData)-s(1).Data(posId,:))*model.kP + s(1).Data(velId,:)*model.kV) ...
            ./ repmat(sIn,model.nbVarPos,1);% ./ repmat((xTar-xStart), 1, nbData);

% dy_0 = s(1).Data(velId, 1);

% figure; plot3(traj_dataset{1}(:, 1), traj_dataset{1}(:, 2), traj_dataset{1}(:, 3), 'b-'); hold on; grid on;
% plot3(s(1).Data(1, :), s(1).Data(2, :), s(1).Data(3, :), 'r-'); 
% 
% figure; plot3(s(1).Data(4, :), s(1).Data(5, :), s(1).Data(6, :), 'b-'); grid on;
% 


%% Display for debug
% x = new_start; % column vector, 3 x 1
% xTar = new_goal; % column vector, 3 x 1
% dx = zeros(3, 1); % column vector, 3 x 1
% for t = 1 : nbData
% 	%Compute acceleration, velocity and position	 
%     ddx = L * [xTar-x; -dx] + f(:,t) * sIn(t) .* (xTar-xStart); 
% 	dx = dx + ddx * dt;
% 	x = x + dx * dt;
% 	y(:,t) = x;
% end


end
