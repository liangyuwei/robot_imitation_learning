function [traj, n_first] = generate_fake_eef_traj(start, via, goal, T)
%% This function generates straight-line trajectory through via-points.
% input start/mid/final should be of size (1 * 6).
% output traj is (T * 6).

global fake_traj_left_or_right

% randomly set the number of points for each stage
% n_first = round(unifrnd(0.5, 0.6) * T);
if fake_traj_left_or_right
    n_first = round(unifrnd(0.6, 0.7) * T);
else
    n_first = round(unifrnd(0.2, 0.3) * T);    
end
n_second = T - n_first;

% generate fake traj
traj = zeros(T, 6);
for i = 1 : 6
    traj(:, i) = [linspace(start(i), via(i), n_first)';...
                  linspace(via(i), goal(i), n_second)'];

end


end