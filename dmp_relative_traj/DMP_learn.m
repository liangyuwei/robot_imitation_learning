%% This file learns and generalizes relative position trajectories using DMP, and store the learned result in .h5 file for later use.
% Targets to encode: elbow pos relative to wrist, relative wrist pos (between left and right). Three parts in total.    

% addpath('./m_fcts/');

%% Load demonstration
file_name = '../motion-retargeting/test_imi_data_YuMi.h5';
group_name = 'fengren_1';

l_wrist_pos = h5read(file_name, ['/', group_name, '/l_wrist_pos']);
l_elbow_pos = h5read(file_name, ['/', group_name, '/l_elbow_pos']);
r_wrist_pos = h5read(file_name, ['/', group_name, '/r_wrist_pos']);
r_elbow_pos = h5read(file_name, ['/', group_name, '/r_elbow_pos']);

lr_wrist_pos = l_wrist_pos - r_wrist_pos;
l_elbow_wrist_pos = l_elbow_pos - l_wrist_pos;
r_elbow_wrist_pos = r_elbow_pos - r_wrist_pos;


%% DMP settings
num_datapoints = 50; % resampling inside DMP_learn.m !!!
nbStates = 64; %Number of activation functions (i.e., number of states in the GMM)
nbVar = 1; %Number of variables for the radial basis functions [s] (decay term)
nbVarPos = 3; %2; %Number of motion variables [x1,x2] 
kP = 50; %Stiffness gain
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
alpha = 1.0; %Decay factor
dt = 1/num_datapoints; %Duration of time step
nbSamples = 1; % Number of samples (of the same movement)
display = true; % display the reproduction to see the performance

%% Construct traj_dataset
% relative position between left and right wrists
traj_dataset{1} = lr_wrist_pos'; % should be of size Length x DOF!!!
[Mu_lrw, Sigma_lrw, Weights_lrw] = DMP_learn_weights(traj_dataset, num_datapoints, ...
                                                     nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, display);


% relative position between elbow and wrist (Left)
traj_dataset{1} = l_elbow_wrist_pos'; % should be of size Length x DOF!!!
[Mu_lew, Sigma_lew, Weights_lew] = DMP_learn_weights(traj_dataset, num_datapoints, ...
                                                     nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, display);

% relative position between elbow and wrist (Right)
traj_dataset{1} = r_elbow_wrist_pos'; % should be of size Length x DOF!!!
[Mu_rew, Sigma_rew, Weights_rew] = DMP_learn_weights(traj_dataset, num_datapoints, ...
                                                     nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, display);



%% Load imitation data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
posId=[1:model.nbVarPos]; velId=[model.nbVarPos+1:2*model.nbVarPos]; accId=[2*model.nbVarPos+1:3*model.nbVarPos]; 
% demos=[];
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
xTar = new_goal; %demos{1}.pos(:,end);
Data=[];
DataDMP=[];
%traj_dataset = ...; % {i} 1000 x 3
for n=1:nbSamples
    tmp = traj_dataset{n}';
%     xStart = tmp(:, 1); 
    xTar = tmp(:, end); % modified by LYW
	%Demonstration data as [x;dx;ddx]
	s(n).Data = spline(1:size(traj_dataset{n}, 1), traj_dataset{n}', linspace(1, size(traj_dataset{n}, 1), nbData)); %Resampling
	s(n).Data = [s(n).Data; gradient(s(n).Data)/model.dt]; %Velocity computation	
	s(n).Data = [s(n).Data; gradient(s(n).Data(end-model.nbVarPos+1:end,:))/model.dt]; %Acceleration computation
	Data = [Data s(n).Data]; %Concatenation of the multiple demonstrations	
    %Nonlinear forcing term
	DataDMP = [DataDMP, (s(n).Data(accId,:) - ...
		(repmat(xTar,1,nbData)-s(n).Data(posId,:))*model.kP + s(n).Data(velId,:)*model.kV) ...
            ./ repmat(sIn,model.nbVarPos,1) ]; %./ repmat(xTar-xStart, 1, nbData)];
end
xTar = new_goal;  % modified by LYW


%% Setting of the basis functions and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_timeBased(sIn, model);

%model = init_GMM_logBased(sIn, model); %Log-spread in s <-> equal spread in t
%model = init_GMM_kmeans(sIn, model);

%Set Sigma as diagonal matrices (i.e., inpedendent systems synchronized by the s variable)
for i=1:model.nbStates
	model.Sigma(:,:,i) = 2E-3; %Setting of covariance
end

%Compute activation
H = zeros(model.nbStates,nbData);
for i=1:model.nbStates
	H(i,:) = gaussPDF(sIn, model.Mu(:,i), model.Sigma(:,:,i));
end
H = H ./ repmat(sum(H),model.nbStates,1);
H2 = repmat(H,1,nbSamples);

% %Nonlinear force profile retrieval (as in RBFN)
% MuF = DataDMP * pinv(H2);

%Nonlinear force profile retrieval (as in LWR)
X = ones(nbSamples*nbData,1);
Y = DataDMP';
for i=1:model.nbStates
	W = diag(H2(i,:));
	MuF(:,i) = (X'*W*X \ X'*W * Y)';
end

%Motion retrieval with DMP
currF = MuF * H; 
x = new_start; %Data(1:model.nbVarPos,1); % starting position!!!
dx = zeros(model.nbVarPos,1);
for t=1:nbData
	%Compute acceleration, velocity and position	 
    ddx = L * [xTar-x; -dx] + currF(:,t) * sIn(t); 
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	r(1).Data(:,t) = x;
end

% save the parameters of the DMP
tmp = struct('name', DMP_name, 'Mu', model.Mu, 'Sigma', model.Sigma, 'Weights', MuF);
if exist('learned_DMP_params.mat', 'file')
    % the file exists, load it
    load learned_DMP_params
    nn = length(DMP_params);
    DMP_params(nn + 1) = tmp;
else
    % the file doesn't exist
    DMP_params = tmp;
end
save learned_DMP_params.mat DMP_params


