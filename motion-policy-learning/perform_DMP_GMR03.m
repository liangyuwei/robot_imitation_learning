function perform_DMP_GMR03(traj_dataset, nbStates, nbVarPos, kP, kV, alpha, dt, nbData, nbSamples, new_goal, new_start)
% Enhanced dynamic movement primitive (DMP) model trained with EM by using a Gaussian mixture 
% model (GMM) representation, with full covariance matrices coordinating the different variables 
% in the feature space. After learning (i.e., autonomous organization of the basis functions 
% (position and spread), Gaussian mixture regression (GMR) is used to regenerate the path of 
% a spring-damper system, resulting in a nonlinear force profile. 
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

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = nbStates; %5; %Number of states in the GMM
model.nbVar = nbVarPos + 1; %3; %Number of variables [s,F1,F2] (decay term and perturbing force)
model.nbVarPos = nbVarPos; %model.nbVar-1; %Dimension of spatial variables
model.kP = kP;%50; %Stiffness gain
model.kV = kV;%(2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = alpha;%1.0; %Decay factor
model.dt = dt; %0.01; %Duration of time step
% nbData = nbData; %200; %Length of each trajectory
% nbSamples = 4; %Number of demonstrations
L = [eye(model.nbVarPos)*model.kP, eye(model.nbVarPos)*model.kV]; %Feedback term
%Create transformation matrix to compute r(1).currTar = x + dx*kV/kP + ddx/kP, see Eq. (4.0.2) in doc/TechnicalReport.pdf
K1d = [1, model.kV/model.kP, 1/model.kP];
K = kron(K1d,eye(model.nbVarPos));


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demos=[];
% load('data/2Dletters/G.mat');
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
Data=[];
DataDMP=[];
for n=1:nbSamples
	%Demonstration data as [x;dx;ddx]
	s(n).Data = spline(1:size(traj_dataset{n}, 1), traj_dataset{n}', linspace(1,size(traj_dataset{n}, 1), nbData)); %Resampling
	s(n).Data = [s(n).Data; gradient(s(n).Data)/model.dt]; %Velocity computation
	s(n).Data = [s(n).Data; gradient(s(n).Data(end-model.nbVarPos+1:end,:))/model.dt]; %Acceleration computation
	DataDMP = [DataDMP [sIn; K*s(n).Data]]; %Training data as [s;r(1).currTar] % g is learnt
	Data = [Data s(n).Data]; %Original data as [x;dx;ddx]
end


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_timeBased(DataDMP, model);
%model = init_GMM_logBased(DataDMP, model); %Log-spread in s <-> equal spread in t
model = init_GMM_kmeans(DataDMP, model);
model = EM_GMM(DataDMP, model);
%Spring-damper attractor path retrieval
r(1).currTar = GMR(model, sIn, 1, 2:model.nbVar);
%Motion retrieval with DMP
x = new_start; %Data(1:model.nbVarPos,1);
dx = zeros(model.nbVarPos,1);
for t=1:nbData
	%Compute acceleration, velocity and position	
	ddx =  L * [r(1).currTar(:,t)-x; -dx]; %Spring-damper system
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	r(1).Data(:,t) = x;
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,450],'color',[1 1 1]); 
xx = round(linspace(1,64,model.nbStates));
clrmap = colormap('jet')*0.5;
clrmap = min(clrmap(xx,:),.9);

%Activation of the basis functions
for i=1:model.nbStates
	h(i,:) = model.Priors(i) * gaussPDF(sIn, model.Mu(1,i), model.Sigma(1,1,i));
end
h = h ./ repmat(sum(h,1)+realmin, model.nbStates, 1);

%Spatial plot
subplot(2,4,[1,5]); hold on; axis off;
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.7 .7 .7]);
plot(r(1).Data(1,:),r(1).Data(2,:),'-','linewidth',3,'color',[.8 0 0]);
axis equal; 

%Timeline plot of the nonlinear perturbing force
subplot(2,4,[2:4]); hold on;
for n=1:nbSamples
	plot(sIn, DataDMP(2,(n-1)*nbData+1:n*nbData), '-','linewidth',2,'color',[.7 .7 .7]);
end
for i=1:model.nbStates
	plotGMM(model.Mu(1:2,i), model.Sigma(1:2,1:2,i), clrmap(i,:), .7);
end
plot(sIn, r(1).currTar(1,:), '-','linewidth',2,'color',[.8 0 0]);
axis([0 1 min(DataDMP(2,:)) max(DataDMP(2,:))]);
ylabel('$\hat{x}_1$','fontsize',16,'interpreter','latex');
view(180,-90);

%Timeline plot of the basis functions activation
subplot(2,4,[6:8]); hold on;
for i=1:model.nbStates
	patch([sIn(1), sIn, sIn(end)], [0, h(i,:), 0], min(clrmap(i,:)+0.5,1), 'EdgeColor', 'none', 'facealpha', .4);
	plot(sIn, h(i,:), 'linewidth', 2, 'color', min(clrmap(i,:)+0.2,1));
end
axis([0 1 0 1]);
xlabel('$s$','fontsize',16,'interpreter','latex'); 
ylabel('$h$','fontsize',16,'interpreter','latex');
view(180,-90);

%print('-dpng','graphs/demo_DMP_GMR03.png');
pause;
close all;
