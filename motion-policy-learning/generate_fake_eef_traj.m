function traj = generate_fake_eef_traj(start, via, goal, T)
%% This function generates straight-line trajectory through via-points.
% input start/mid/final should be of size (1 * 6).
% output traj is (T * 6).

% randomly set the number of points for each stage
n_first = round(unifrnd(0.5, 0.6) * T);
n_second = T - n_first;

% generate fake traj
traj = zeros(T, 6);
for i = 1 : 6
    traj(:, i) = [linspace(start(i), via(i), n_first)';...
                  linspace(via(i), goal(i), n_second)'];

end


end