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

%Create transformation matrix to compute r(1).currTar = x + dx*kV/kP + ddx/kP
K1d = [1, model.kV/model.kP, 1/model.kP];
K = kron(K1d,eye(model.nbVarPos));


%% Load imitation data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
posId=1:model.nbVarPos; velId=model.nbVarPos+1:2*model.nbVarPos; accId=2*model.nbVarPos+1:3*model.nbVarPos; 

% sIn(1) = 1; %Initialization of decay term
% for t=2:nbData
% 	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
% end
sIn = exp(-model.alpha * (0:nbData-1)*model.dt);


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

    %Nonlinear forcing term - for methods modeling f/x/(g-y0) directly
	%
    DataDMP = [DataDMP, (s(n).Data(accId,:) - ...
		(repmat(xTar,1,nbData)-s(n).Data(posId,:))*model.kP + s(n).Data(velId,:)*model.kV) ...
            ./ repmat(sIn,model.nbVarPos,1) ./ repmat(xTar-xStart, 1, nbData)];
    %}

    %Training data as [s;F] - for methods like GMR which relates time and f/x/(g-y0)         
	%{
    DataDMP = [DataDMP [sIn; ...
		(s(n).Data(accId,:) - ...
        (repmat(xTar,1,nbData)-s(n).Data(posId,:))*model.kP + s(n).Data(velId,:)*model.kV) ...
        ./ repmat(sIn,model.nbVarPos,1) ./ repmat(xTar-xStart, 1, nbData) ] ];
    %}
    
    %Training data as [s;r(1).currTar]
    %DataDMP = [DataDMP [sIn; K*s(n).Data]]; 
           
end
% xTar = new_goal;  % modified by LYW


%% Setting of the basis functions and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for methods that models force f/x/(g-y0) directly
model = init_GMM_timeBased(sIn, model);
% model = init_GMM_logBased(sIn, model); %Log-spread in s <-> equal spread in t
for i=1:model.nbStates
	model.Sigma(:,:,i) = 1E-3; %Setting of covariance
end


% for methods like GMR that relate time and force f/x/(g-y0)
% 1 - GMR01
% model = init_GMM_timeBased(DataDMP, model); 
% 2 - GMR02, GMR03
% model = init_GMM_kmeans(DataDMP, model);
% model = EM_GMM(DataDMP, model);


%Compute activation
H = zeros(model.nbStates,nbData);
for i=1:model.nbStates
	H(i,:) = gaussPDF(sIn, model.Mu(:,i), model.Sigma(:,:,i));
end
H = H ./ repmat(sum(H),model.nbStates,1);
H2 = repmat(H,1,nbSamples);


%% Learn weights(MuF) and retrieve the spatiotemporal-independent nonlinear force profile(Yr) (different ways of modeling it)
% 1 - Nonlinear force profile retrieval (as in standard DMP)
% Best performance when modeling a single, complex trajectory sample !!!      
MuF = DataDMP * pinv(H2);
Yr = (MuF * H)'; 


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


% 3 - Nonlinear force profile retrieval (as in Ijspeert'13; (g-y0) modulation added by LYW)   
%{
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
%}
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
% Be sure to modify the preparation of DataDMP, repro and the visualization
% code.
%{
G = zeros(nbData * nbSamples, model.nbVarPos);
for n = 1 : nbSamples
    xTar = traj_dataset{1}(end, :)';
    xStart = traj_dataset{1}(1, :)';
    G((n-1)*nbData+1 : n*nbData, :) = repmat((xTar-xStart)', nbData, 1);
end
test_tmp = 1;
xTar = traj_dataset{test_tmp}(end, :)';
xStart = traj_dataset{test_tmp}(1, :)';
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

% 6 - Nonlinear force profile retrieval - WLS version 2 (with polynomial)
%{
for i=1:model.nbStates
	st(i).X = [];
	st(i).Xr = [];
	for d=0:model.polDeg 
		st(i).X = [st(i).X, repmat((sIn-model.Mu(1,i)).^d,1,nbSamples)'];
		st(i).Xr = [st(i).Xr, (sIn-model.Mu(1,i)).^d'];
	end
end
Y = DataDMP';
for i=1:model.nbStates
	W = diag(H2(i,:));
	MuF(:,:,i) = st(i).X' * W * st(i).X \ st(i).X' * W * Y; %Weighted least squares
end
Yr = zeros(nbData,model.nbVarPos);
for t=1:nbData
	for i=1:model.nbStates
		Yr(t,:) = Yr(t,:) + H(i,t) * st(i).Xr(t,:) * MuF(:,:,i);
	end
end
%}

% 7 - GMR, modeling the relationship between
% currF = GMR(model, sIn, 1, 2:model.nbVarPos+1);
% Yr = currF';


% 8 - GMR, use with init_GMM_kmeans and EM_GMM initialization
% learn a goal signal for leading to trajectory generation
% r(1).currTar = GMR(model, sIn, 1, 2:model.nbVarPos+1);



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
    xTar = traj_dataset{1}(end, :)';% + [0, -0.5, 1]';
    xStart = traj_dataset{1}(1, :)';% + [0, 0.5, 1]';
    x = xStart; %Data(1:model.nbVarPos,1);
    dx = zeros(model.nbVarPos,1);
    repro = [];
    for t=1:nbData
        %Compute acceleration, velocity and position
        ddx = L * [xTar-x; -dx] + Yr(t,:)' * sIn(t) .* (xTar - xStart);
        %ddx = L * [r(1).currTar(:,t)-x; -dx]; %Spring-damper system % for use with GMR03
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


figure;
sgtitle('Reproduced and original nonlinear force profile');
subplot(3, 1, 1); plot(DataDMP(1, :), 'b-'); hold on; grid on;
plot(Yr(:, 1), 'r-'); 
ylabel('X Force'); title('x');
subplot(3, 1, 2); plot(DataDMP(2, :), 'b-'); hold on; grid on;
plot(Yr(:, 2), 'r-'); 
ylabel('Y Force'); title('y');
subplot(3, 1, 3); plot(DataDMP(3, :), 'b-'); hold on; grid on;
plot(Yr(:, 3), 'r-'); 
ylabel('Z Force'); title('z');


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
