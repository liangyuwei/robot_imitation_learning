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


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_datapoints = 50;
model.nbStates = 8; %5; %Number of activation functions (i.e., number of states in the GMM)
model.nbVar = 1; %Number of variables for the radial basis functions [s] (decay term)
model.nbVarPos = 3; %2; %Number of motion variables [x1,x2] 
model.kP = 50; %Stiffness gain
model.kV = (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = 1.0; %Decay factor
model.dt = 1/num_datapoints; %Duration of time step
nbData = num_datapoints; %200; %Length of each trajectory
nbSamples = 1; %Number of demonstrations
L = [eye(model.nbVarPos)*model.kP, eye(model.nbVarPos)*model.kV]; %Feedback term


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
posId=[1:model.nbVarPos]; velId=[model.nbVarPos+1:2*model.nbVarPos]; accId=[2*model.nbVarPos+1:3*model.nbVarPos]; 
% demos=[];
% load('data/2Dletters/G.mat');
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


%% Plots
%{
% plot DataDMP(force profile) and currF(retrieved force)
trajId = 1;
figure;
plot3(0, 0, 0, 'ro'); hold on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');
title(['Imi Traj ', num2str(trajId)]);
for i = trajId : trajId%nbSamples
    for j = 1 : nbData
        plot3(DataDMP(1, (i-1)*nbData+j), DataDMP(2, (i-1)*nbData+j), DataDMP(3, (i-1)*nbData+j), 'b.');
        pause(0.01);
    end
end

% plot velocity component of the imitation trajectory
figure;
plot3(0, 0, 0, 'ro'); hold on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');
title('Vel component of imitation trajectory');
plot3(s(n).Data(7, :), s(n).Data(8, :), s(n).Data(9, :), 'b-');

% plot acceleration component of the imitation trajectory
figure;
plot3(0, 0, 0, 'ro'); hold on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');
title('Acc component of imitation trajectory');
plot3(s(n).Data(13, :), s(n).Data(14, :), s(n).Data(15, :), 'b-');

% DMP
figure;
plot3(0, 0, 0, 'ro'); hold on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');
title('CurrF');
plot3(currF(1, 1), currF(2, 1), currF(3, 1), 'go');
plot3(currF(1, end), currF(2, end), currF(3, end), 'ro');
for j = 2 : nbData-1
    plot3(currF(1, j), currF(2, j), currF(3, j), 'b.');
    pause(0.01);
end
% plot Data
figure;
plot3(Data(1, :), Data(2, :), Data(3, :), 'b.'); grid on;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 16 4],'position',[10,10,1300,500],'color',[1 1 1]); 
xx = round(linspace(1,64,model.nbStates));
clrmap = colormap('jet')*0.5;
clrmap = min(clrmap(xx,:),.9);

%Spatial plot
axes('Position',[0 0 .2 1]); hold on; axis off;
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.7 .7 .7]);
plot(r(1).Data(1,:),r(1).Data(2,:),'-','linewidth',3,'color',[.8 0 0]);
axis equal; axis square;  

%Timeline plot of the nonlinear perturbing force
axes('Position',[.25 .58 .7 .4]); hold on; 
for n=1:nbSamples
	plot(sIn, DataDMP(1,(n-1)*nbData+1:n*nbData), '-','linewidth',2,'color',[.7 .7 .7]);
end
[~,id] = max(H);
for i=1:model.nbStates
	plot(sIn(id==i), repmat(MuF(1,i),1,sum(id==i)), '-','linewidth',6,'color',min(clrmap(i,:)+0.5,1));
end
plot(sIn, currF(1,:), '-','linewidth',2,'color',[.8 0 0]);
axis([min(sIn) max(sIn) min(DataDMP(1,:)) max(DataDMP(1,:))]);
ylabel('$F_1$','fontsize',16,'interpreter','latex');
view(180,-90);

%Timeline plot of the basis functions activation
axes('Position',[.25 .12 .7 .4]); hold on; 
for i=1:model.nbStates
	patch([sIn(1), sIn, sIn(end)], [0, H(i,:), 0], min(clrmap(i,:)+0.5,1), 'EdgeColor', 'none', 'facealpha', .4);
	plot(sIn, H(i,:), 'linewidth', 2, 'color', min(clrmap(i,:)+0.2,1));
end
axis([min(sIn) max(sIn) 0 1]);
xlabel('$s$','fontsize',16,'interpreter','latex'); 
ylabel('$h$','fontsize',16,'interpreter','latex');
view(180,-90);
%}
%print('-dpng','graphs/demo_DMP01.png');
%pause;
%close all;
