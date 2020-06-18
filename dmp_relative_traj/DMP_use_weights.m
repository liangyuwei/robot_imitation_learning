function repro_final = DMP_use_weights(Mu, Sigma, Weights, nbData, kP, kV, alpha, dt, xTar, xStart, display)
%% This function takes the learned weights of DMP, and generates a new trajectory given the new start, new goal and specified trajectory length.
% Actually, nonlinear force retrieval and motion retrieval have been
% implemented and tested in DMP_learn_weights.m, e.g. LWR, direct inverse,
% polynomial fitting, this function is only to provide an interface for
% generating new trajectory given new g, y0 and nbData!!!

% Input: Mu - of size (1 x nbStates)
%        Sigma - of size (1 x 1 x  nbStates)
%        Weights(MuF) - of size (nbVarPos x nbStates), nbStates weights for each DOF       
%        nbData - expected length of the reproduced trajectory
%        xTar, xStart - should be column vector, of size (nbVarPos x 1)

%% Preparation 
% meta-info
[nbVarPos, nbStates] = size(Weights);
L = [eye(nbVarPos)*kP, eye(nbVarPos)*kV]; %Feedback term

% internal clock (following the setting for learning)
train_nbData = 400;
sIn = exp(-alpha * (0:train_nbData-1)*dt);

%Compute activation
H = zeros(nbStates,train_nbData);
for i=1:nbStates
	H(i,:) = gaussPDF(sIn, Mu(:,i), Sigma(:,:,i));
end
H = H ./ repmat(sum(H),nbStates,1);
% H2 = repmat(H,1,nbSamples);


%% Retrieval of spatiotemporal-independent nonlinear force profile
MuF = Weights;

% 1 - Nonlinear force profile retrieval (as in standard DMP)
% Best performance when modeling a single, complex trajectory sample !!!      
% MuF = DataDMP * pinv(H2);
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


%% Motion retrieval with DMP
% xTar = traj_dataset{1}(end, :)';% + [0, -0.5, 1]';
% xStart = traj_dataset{1}(1, :)';% + [0, 0.5, 1]';
x = xStart; %Data(1:model.nbVarPos,1);
dx = zeros(nbVarPos,1);
repro = [];
for t=1:train_nbData
    %Compute acceleration, velocity and position
    ddx = L * [xTar-x; -dx] + Yr(t,:)' * sIn(t) .* abs(xTar - xStart);
    %ddx = L * [r(1).currTar(:,t)-x; -dx]; %Spring-damper system % for use with GMR03
    dx = dx + ddx * dt;
    x = x + dx * dt;
    repro(:,t) = x;
end


%% Convert to a path with required length
repro_final = zeros(nbVarPos, nbData);
% repro_final(1, :) = interp1(sIn, repro(1, :), linspace(sIn(1), sIn(end), nbData));
% repro_final(2, :) = interp1(sIn, repro(2, :), linspace(sIn(1), sIn(end), nbData));
% repro_final(3, :) = interp1(sIn, repro(3, :), linspace(sIn(1), sIn(end), nbData));
% sIn is not linear in time !!! The interpolation result using sIn is weird, denser in the beginning and sparser in the final stage.
repro_final(1, :) = interp1(linspace(1, 0, train_nbData), repro(1, :), linspace(1, 0, nbData));
repro_final(2, :) = interp1(linspace(1, 0, train_nbData), repro(2, :), linspace(1, 0, nbData));
repro_final(3, :) = interp1(linspace(1, 0, train_nbData), repro(3, :), linspace(1, 0, nbData));


% debug display
if display
    figure;
    plot3(repro(1, :), repro(2, :), repro(3, :), 'b.'); hold on; grid on;
    plot3(repro_final(1, :), repro_final(2, :), repro_final(3, :), 'r*');
end

end