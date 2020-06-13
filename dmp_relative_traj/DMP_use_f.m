function y = DMP_use_f(f, nbData, alpha, kP, kV, new_goal, new_start)
%% This function generates
% y, f are of the size DOF(3) x Length(50)

dt = 1/15; %1/nbData*2;
L = [eye(3)*kP, eye(3)*kV]; %Feedback term

%% Initialization of decay term
sIn(1) = 1; 
for t=2:nbData
	sIn(t) = sIn(t-1) - alpha * sIn(t-1) * dt; %Update of decay term (ds/dt=-alpha s)
end
% sIn = linspace(1, 0, nbData);

%% Motion retrieval
x = new_start; % column vector, 3 x 1
xTar = new_goal; % column vector, 3 x 1
dx = zeros(3, 1); %dy_0; %
for t=1:nbData
	%Compute acceleration, velocity and position	 
    ddx = L * [xTar-x; -dx] + f(:,t) * sIn(t);% .* (new_goal-new_start); 
	dx = dx + ddx * dt;
	x = x + dx * dt;
	y(:,t) = x;
end


end