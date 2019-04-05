%% Operations on covariance of multivariate random variables
% Generate three signals
% normal_sig1 = normrnd(0, 5, 1000, 1); %plot(1:length(normal_sig1), normal_sig1, 'b.'); 
% normal_sig2 = normrnd(-0.5, 3, 1000, 1);
% normal_sig3 = normrnd(0.5, 8, 1000, 1);
%
mu1 = [1, 2, 6, -1, -2, -6]; Sigma1 = [2, 0.5, 3, 2, 0.5, 3];%[2, 0, 0; 0, 0.5, 0; 0, 0, 3];
mu2 = [-3, -5, 4, 3, 5, -4]; Sigma2 = [1, 1, 2, 1, 1, 2]; %[1, 0, 0; 0, 1, 0; 0, 0, 2];
X = [mvnrnd(mu1, Sigma1, 1000); mvnrnd(mu2, Sigma2, 1000)]; % two 1000 x 2 signals

% obtain GMM model
% GM1 = fitgmdist([normal_sig1, normal_sig2, normal_sig3], 10);
GM2 = fitgmdist(X, 2);

%% Normal probability density function
y = @(x) normpdf(x, [0, 1, 5, 10], [0.1, 0.5, 0.2, 0.01]);
figure; hold on;
for t = -10:0.01:10
    plot(t, y(t), 'b.');
end
axis([-10, 10, 0, 1]);


%% Multivariate normal density function
tmp = mvnpdf([0, 0], [-1, -1], [0.1, 0; 0, 0.1])


%% Linear interpolation
xyz = [0,0,0; 5,10,5; 10,-10,10];
figure; plot3(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'b-'); grid on;
xyz_interpl = [ xyz(1, 1) : (xyz(2, 1) - xyz(1, 1)) / 10 : xyz(2, 1), xyz(2, 1) : (xyz(3, 1) - xyz(2, 1)) / 10 : xyz(3, 1);
                xyz(1, 2) : (xyz(2, 2) - xyz(1, 2)) / 10 : xyz(2, 2), xyz(2, 2) : (xyz(3, 2) - xyz(2, 2)) / 10 : xyz(3, 2);
                xyz(1, 3) : (xyz(2, 3) - xyz(1, 3)) / 10 : xyz(2, 3), xyz(2, 3) : (xyz(3, 3) - xyz(2, 3)) / 10 : xyz(3, 3)];
figure; 
plot3(xyz_interpl(1, :), xyz_interpl(2, :), xyz_interpl(3, :), 'r.'); grid on;


%% Frechet distance, from online
% create data
t = 0:pi/8:2*pi;
y = linspace(1,3,6);
P = [(2:7)' y']+0.3.*randn(6,2);
Q = [t' sin(t')]+2+0.3.*randn(length(t),2);
[cm, cSq] = DiscreteFrechetDist(P,Q);
% plot result
figure
plot(Q(:,1),Q(:,2),'o-r','linewidth',3,'markerfacecolor','r')
hold on
plot(P(:,1),P(:,2),'o-b','linewidth',3,'markerfacecolor','b')
title(['Discrete Frechet Distance of curves P and Q: ' num2str(cm)])
legend('Q','P','location','best')
line([2 cm+2],[0.5 0.5],'color','m','linewidth',2)
text(2,0.4,'dFD length')
for i=1:length(cSq)
  line([P(cSq(i,1),1) Q(cSq(i,2),1)],...
       [P(cSq(i,1),2) Q(cSq(i,2),2)],...
       'color',[0 0 0]+(i/length(cSq)/1.35));
end
axis equal
% display the coupling sequence along with each distance between points
disp([cSq sqrt(sum((P(cSq(:,1),:) - Q(cSq(:,2),:)).^2,2))])


