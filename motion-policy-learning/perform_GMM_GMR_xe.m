function xe_out = perform_GMM_GMR_xe(Data, nbStates, nbVar, dt, nbData, nbSamples, xw_in)
% xw_in - 3-dim trajectory as input of GMR
% Data - 6-dim, including xw and xe datasets(aligned)

% 3D visualization of a Gaussian mixture model (GMM) with time-based Gaussian mixture 
% regression (GMR) used for reproduction
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
model.nbVar = nbVar; %6; %Number of variables [xw_dataset_aligned(3), xe_dataset_aligned(3)]
model.dt = dt; % NEEDED ANY MORE??? 0.01; %Time step duration
nbData = nbData; %100; %Length of each trajectory
nbSamples = nbSamples; %5; %Number of demonstrations


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demos=[];
% load('data/2Dletters/G.mat'); %Load x1,x2 variables
% for n=1:nbSamples
% 	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
% end
% demos=[];
% load('data/2Dletters/C.mat'); %Load x3,x4 variables
% Data=[];
% for n=1:nbSamples
% 	s(n).Data = [[1:nbData]*model.dt; s(n).Data; spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData))]; %Resampling
% 	Data = [Data s(n).Data]; 
% end
Data = [];


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_kmeans(Data, model);
model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
% 1:3 is xw_dataset_aligned, 4:6 is xe_dataset_aligned, xw_in is the input to GMR     
[DataOut, SigmaOut] = GMR(model, xw_in, 1:3, 4:6); %see Eq. (17)-(19)


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Display figure (can take some time)...');
figure('position',[10,10,1000,500]); 
%Plot GMM
subplot(1,2,1); hold on; box on; title('GMM');
plotGMM3D(model.Mu(2:4,:), model.Sigma(2:4,2:4,:), [.8 0 0], .3);
%plot3(Data(2,:),Data(3,:),Data(4,:),'.','markersize',8,'color',[.7 .7 .7]);
for n=1:nbSamples
	dTmp = [Data(2:4,(n-1)*nbData+1:n*nbData) fliplr(Data(2:4,(n-1)*nbData+1:n*nbData))];
	patch(dTmp(1,:),dTmp(2,:),dTmp(3,:), [.5,.5,.5],'facealpha',0,'linewidth',2,'edgecolor',[.5,.5,.5],'edgealpha',.5);
end
view(3); axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]); set(gca,'Ztick',[]);
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
%Plot GMR
subplot(1,2,2); hold on; box on; title('GMR');
plotGMM3D(DataOut(1:3,1:2:end), SigmaOut(1:3,1:3,1:2:end), [0 .8 0], .2, 2);
for n=1:nbSamples
	dTmp = [Data(2:4,(n-1)*nbData+1:n*nbData) fliplr(Data(2:4,(n-1)*nbData+1:n*nbData))];
	patch(dTmp(1,:),dTmp(2,:),dTmp(3,:), [.5,.5,.5],'facealpha',0,'linewidth',2,'edgecolor',[.5,.5,.5],'edgealpha',.5);
end
plot3(DataOut(1,:),DataOut(2,:),DataOut(3,:),'-','linewidth',4,'color',[0 .4 0]);
view(3); axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]); set(gca,'Ztick',[]);
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');

%print('-dpng','graphs/demo_GMR_3Dviz01.png');
%pause;
%close all;
