function traj = gen_fake_traj(start, goal, offset)
% traj - 1000 * 3

% set via point
via = goal;
via(2) = offset;

% generate fake traj
n_first = round(unifrnd(0.7, 0.9) * 1000);
n_second = 1000 - n_first;
traj = [linspace(start(1), via(1), n_first)', linspace(start(2), via(2), n_first)', linspace(start(3), via(3), n_first)';...
        linspace(via(1), goal(1), n_second)', linspace(via(2), goal(2), n_second)', linspace(via(3), goal(3), n_second)'];


end