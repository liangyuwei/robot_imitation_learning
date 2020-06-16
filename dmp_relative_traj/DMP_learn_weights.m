function [Mu, Sigma, Weights] = DMP_learn_weights(traj_dataset, nbData, nbStates, nbVar, nbVarPos, kP, kV, alpha, nbSamples, display)
%% This function trains a DMP on the imitation trajectory
% addpath('./m_fcts/');
% input trajectory should be of size Length x DOF


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% num_datapoints = 50;
model.nbStates = nbStates; %8; %5; %Number of activation functions (i.e., number of states in the GMM)
model.nbVar = nbVar; %1; %Number of variables for the radial basis functions [s] (decay term)
model.nbVarPos = nbVarPos;%3; %2; %Number of motion variables [x1,x2] 
model.kP = kP;%50; %Stiffness gain
model.kV = kV; %(2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = alpha; %1.0; %Decay factor
model.dt = 0.01; %1/nbData; %Duration of time step
model.polDeg = 3; %Degree of polynomial fit

% nbData = num_datapoints; %200; %Length of each trajectory
% nbSamples = nbSamples; %1; %Number of demonstrations
L = [eye(model.nbVarPos)*model.kP, eye(model.nbVarPos)*model.kV]; %Feedback term


%% Load imitation data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
posId=[1:model.nbVarPos]; velId=[model.nbVarPos+1:2*model.nbVarPos]; accId=[2*model.nbVarPos+1:3*model.nbVarPos]; 
% demos=[];
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
% decay term, wish it to be in consistent with the time_range on which pass_time is specified..   

Data=[];
DataDMP=[];
%traj_dataset = ...; % {i} 1000 x 3
for n=1:nbSamples
    tmp = traj_dataset{n}';
    xStart = tmp(:, 1); 
    xTar = tmp(:, end); % modified by LYW, column vector, 3 x 1
	%Demonstration data as [x;dx;ddx]
	s(n).Data = spline(1:size(traj_dataset{n}, 1), traj_dataset{n}', linspace(1, size(traj_dataset{n}, 1), nbData)); %Resampling
	s(n).Data = [s(n).Data; gradient(s(n).Data)/model.dt]; %Velocity computation	
	s(n).Data = [s(n).Data; gradient(s(n).Data(end-model.nbVarPos+1:end,:))/model.dt]; %Acceleration computation
	Data = [Data s(n).Data]; %Concatenation of the multiple demonstrations	
    %Nonlinear forcing term
	DataDMP = [DataDMP, (s(n).Data(accId,:) - ...
		(repmat(xTar,1,nbData)-s(n).Data(posId,:))*model.kP + s(n).Data(velId,:)*model.kV) ...
            ./ repmat(sIn,model.nbVarPos,1) ./ repmat(xTar-xStart, 1, nbData)];
end
% xTar = new_goal;  % modified by LYW


%% Setting of the basis functions and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_timeBased(sIn, model);
%model = init_GMM_logBased(sIn, model); %Log-spread in s <-> equal spread in t
% model = init_GMM_kmeans(sIn, model);
% model = EM_GMM(sIn, model);
%Set Sigma as diagonal matrices (i.e., inpedendent systems synchronized by the s variable)
for i=1:model.nbStates
	model.Sigma(:,:,i) = 1E-3; %Setting of covariance
end

%Compute activation
H = zeros(model.nbStates,nbData);
for i=1:model.nbStates
	H(i,:) = gaussPDF(sIn, model.Mu(:,i), model.Sigma(:,:,i));
end
H = H ./ repmat(sum(H),model.nbStates,1);
H2 = repmat(H,1,nbSamples);


%% Learn weights(MuF) and retrieve the spatiotemporal-independent nonlinear force profile(Yr) (different ways of modeling it)
% 1 - Nonlinear force profile retrieval (as in standard DMP)
% MuF = DataDMP * pinv(H2);
% Yr = (MuF * H)'; 


% 2 - Nonlinear force profile retrieval for polynomial of degree 0 (as in demo_DMP01.m)   
% Use 1's as input for regression of the nonliear force profile       
% X = ones(nbSamples*nbData,1); 
% Xr = ones(nbData,1); 
% Y = DataDMP';
% for i=1:model.nbStates
% 	W = diag(H2(i,:));
% 	MuF(:,i) = X'*W*X \ X'*W * Y; %Weighted least squares
% end
% Yr = (MuF * H)'; 


% 3 - Nonlinear force profile retrieval (as in Ijspeert'13, but without (g-y0) modulation)
X = repmat(sIn',nbSamples,1);
Y = DataDMP';
for i=1:model.nbStates
	W = diag(H2(i,:));
	MuF(:,i) = X'*W*X \ X'*W * Y; %Weighted least squares
end
Xr = sIn';
Yr = zeros(nbData,model.nbVarPos);
for i=1:model.nbStates
	Yr = Yr + diag(H(i,:)) * Xr * MuF(:,i)';
end
% for t=1:nbData
% 	for i=1:model.nbStates
% 		Yr(t,:) = Yr(t,:) + H(i,t) * Xr(t,:) * MuF(:,i)';
% 	end
% end


% 4 - Nonlinear force profile retrieval (as in Ijspeert'13, including (g-y0) modulation)
% by LYW, 2020/06/16: Since we have processed f_d to exclude temporal and spatial variables, this way should not be used.    
% this method originally used the same start and goal for all the samples... which is not quite reasonable... so I modified it...   
% The result is the same as performing (g-y0) modulation ahead, as in generating f_d/x/(g-y0)   
% actually, even worse, see the 1st sample of W letter...
%{
G = zeros(nbData * nbSamples, model.nbVarPos);
for n = 1 : nbSamples
    xTar = demos{n}.pos(:,end);
    xStart = demos{n}.pos(:,1);
    G((n-1)*nbData+1 : n*nbData, :) = repmat((xTar-xStart)', nbData, 1);
end
test_tmp = 3;
xTar = demos{test_tmp}.pos(:,end);
xStart = demos{test_tmp}.pos(:,1);
G_new = xTar - xStart;
% G = xTar - xStart; %xTar - Data(1:model.nbVarPos,1);
Yr = zeros(nbData,model.nbVarPos);
for j=1:model.nbVarPos
	X = repmat(sIn',nbSamples,1) .* G(:, j); %.* repmat(G(j),1,nbData*nbSamples)'; % 
	Y = DataDMP(j,:)';
	for i=1:model.nbStates
		W = diag(H2(i,:));
		MuF(j,i) = X'*W*X \ X'*W * Y; %Weighted least squares
	end
	Xr = sIn' .* repmat(G_new(j),1,nbData)';
	for i=1:model.nbStates
		Yr(:,j) = Yr(:,j) + diag(H(i,:)) * Xr * MuF(j,i)';
	end
end
%}


% 5 - Nonlinear force profile retrieval - WLS version 1 (with polynomial)
% still have scale variance issue, this should be due to different force profiles in different demonstrations       
%{
X = [];
Xr = [];
for d=0:model.polDeg 
	X = [X, repmat(sIn.^d,1,nbSamples)'];
	Xr = [Xr, sIn.^d'];
end
Y = DataDMP';
for i=1:model.nbStates
	W = diag(H2(i,:));
	MuF(:,:,i) = X'*W*X \ X'*W * Y; %Weighted least squares
end
Yr = zeros(nbData,model.nbVarPos);
for t=1:nbData
	for i=1:model.nbStates
		Yr(t,:) = Yr(t,:) + H(i,t) * Xr(t,:) * MuF(:,:,i);
	end
end
%}

% %Nonlinear force profile retrieval - WLS version 2 (with polynomial)
% for i=1:model.nbStates
% 	st(i).X = [];
% 	st(i).Xr = [];
% 	for d=0:model.polDeg 
% 		st(i).X = [st(i).X, repmat((sIn-model.Mu(1,i)).^d,1,nbSamples)'];
% 		st(i).Xr = [st(i).Xr, (sIn-model.Mu(1,i)).^d'];
% 	end
% end
% Y = DataDMP';
% for i=1:model.nbStates
% 	W = diag(H2(i,:));
% 	MuF(:,:,i) = st(i).X' * W * st(i).X \ st(i).X' * W * Y; %Weighted least squares
% end
% Yr = zeros(nbData,model.nbVarPos);
% for t=1:nbData
% 	for i=1:model.nbStates
% 		Yr(t,:) = Yr(t,:) + H(i,t) * st(i).Xr(t,:) * MuF(:,:,i);
% 	end
% end


%% Return learned results
Mu = model.Mu;
Sigma = model.Sigma;
Weights = MuF;


%% Motion retrieval with DMP
if display
    %Motion retrieval with DMP
    %     currF = MuF * H;
    %     x = traj_dataset{n}(1, :)'; % the original start
    %     xTar = traj_dataset{n}(end, :)';
    %     dx = zeros(model.nbVarPos,1); % initial velocity
    %     for t=1:nbData
    %         %Compute acceleration, velocity and position
    %         ddx = L * [xTar-x; -dx] + currF(:,t) * sIn(t);
    %         dx = dx + ddx * model.dt;
    %         x = x + dx * model.dt;
    %         repro(:,t) = x;
    %     end
    xTar = traj_dataset{1}(end, :)';
    xStart = traj_dataset{1}(1, :)';
    x = xStart; %Data(1:model.nbVarPos,1);
    dx = zeros(model.nbVarPos,1);
    repro = [];
    for t=1:nbData
        %Compute acceleration, velocity and position
        ddx = L * [xTar-x; -dx] + Yr(t,:)' * sIn(t) .* (xTar - xStart);
        dx = dx + ddx * model.dt;
        x = x + dx * model.dt;
        repro(:,t) = x;
    end
end

figure;
plot3(Data(1, :), Data(2, :), Data(3, :), 'b.'); hold on; grid on;
plot3(repro(1, :), repro(2, :), repro(3, :), 'r*');
title('Reproduced trajectory and the original trajectory');
xlabel('x'); ylabel('y'); zlabel('z'); 


% save the parameters of the DMP
% tmp = struct('name', DMP_name, 'Mu', model.Mu, 'Sigma', model.Sigma, 'Weights', MuF);
% if exist('learned_DMP_params.mat', 'file')
%     % the file exists, load it
%     load learned_DMP_params
%     nn = length(DMP_params);
%     DMP_params(nn + 1) = tmp;
% else
%     % the file doesn't exist
%     DMP_params = tmp;
% end
% save learned_DMP_params.mat DMP_params

end
