function r = generate_path_using_primitives(DMP_name, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, new_goal, new_start)
%% This function generates actions using DMP primitives from the library, with the given new start and new goal.

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = nbStates; %5; %Number of activation functions (i.e., number of states in the GMM)
model.nbVar = nbVar; % 1; %Number of variables for the radial basis functions [s] (decay term)
model.nbVarPos = nbVarPos; %2; %Number of motion variables [x1,x2] 
model.kP = kP; % 50; %Stiffness gain
model.kV = kV; % (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = alpha; %1.0; %Decay factor
model.dt = dt; %0.01; %Duration of time step
L = [eye(model.nbVarPos)*model.kP, eye(model.nbVarPos)*model.kV]; %Feedback term


%% Load the learned parameters for generating new trajectory
% load the params from recorded mat file
load learned_DMP_params.mat
for j = 1 : length(DMP_params)
    if isequal(DMP_params(j).name, DMP_name)
        model.Sigma = DMP_params(j).Sigma;
        model.Mu = DMP_params(j).Mu;
        MuF = DMP_params(j).Weights;
    end
end


%% Motion retrieval with DMP
% reset the clock
% sIn_tmp = 1; sIn = [];
sIn(1) = 1; %Initialization of decay term
nbData = 1000;
for t = 2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
% set the start and goal
x = new_start; xTar = new_goal;
% set initial velocity
dx = zeros(model.nbVarPos,1); 
% set initial activation 
H = zeros(model.nbStates, 1);
% store the result
r.Data = [];
% start iteration to generate new path
threshold = 0.001;
% while (sIn_tmp >= threshold) % wait until both DMP sequences reach the end 
for t = 1 : nbData
    
    sIn_tmp = sIn(t);
    
    % update H, activation
    for i = 1:model.nbStates
        H(i,:) = gaussPDF(sIn_tmp, model.Mu(:,i), model.Sigma(:,:,i));
    end
    H = H ./ repmat(sum(H),model.nbStates,1);
    currF = MuF * H; 

	%Compute acceleration, velocity and position	 
    ddx = L * [xTar - x; -dx] + currF * sIn_tmp; 
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
    r.Data = [r.Data, x]; 

    % record timestamps and set the next time stamp and add coupling terms!!!
%     sIn = [sIn, sIn_tmp];
%     sIn_tmp = sIn_tmp - model.alpha * sIn_tmp * model.dt; %%%%% make the clock of the left arm faster!!! %%%%    
    
end

% for debug
% pause;

