function r = perform_DMP_using_learned_params(DMP_name, nbStates, nbVar, nbVarPos, kP, kV, alpha, dt, nbData, nbSamples, new_goal, new_start)
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
model.nbStates = nbStates; %5; %Number of activation functions (i.e., number of states in the GMM)
model.nbVar = nbVar; % 1; %Number of variables for the radial basis functions [s] (decay term)
model.nbVarPos = nbVarPos; %2; %Number of motion variables [x1,x2] 
model.kP = kP; % 50; %Stiffness gain
model.kV = kV; % (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = alpha; %1.0; %Decay factor
model.dt = dt; %0.01; %Duration of time step
nbData = nbData; %200; %Length of each trajectory
nbSamples = nbSamples; %4; %Number of demonstrations
L = [eye(model.nbVarPos)*model.kP, eye(model.nbVarPos)*model.kV]; %Feedback term

% try other settings
% nbData = 500; % doesn't work since sIn is not enough... and in that Mu and Sigma are still away       

%% Pre-setup
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
xTar = new_goal;  % modified by LYW


%% Load the learned parameters for generating new trajectory
% load the params from recorded mat file
load learned_DMP_params.mat
for j = 1 : length(DMP_params)
    if DMP_params(j).name == DMP_name
        model.Sigma = DMP_params(j).Sigma;
        model.Mu = DMP_params(j).Mu;
        MuF = DMP_params(j).Weights;
    end 
end

%Compute activation
H = zeros(model.nbStates,nbData);
for i=1:model.nbStates
	H(i,:) = gaussPDF(sIn, model.Mu(:,i), model.Sigma(:,:,i));
end
H = H ./ repmat(sum(H),model.nbStates,1);

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
