function Q = merge_two_trajs(Q1, Q2)
%% This function merges two trajectories, each with different time stamp.

% Prep
n_samples = size(Q1, 1);
Q = cell(n_samples, 3);

% Start iterations
for i = 1 : n_samples
    % Merge the time stamps first
    t1 = Q1{i, 2};
    t2 = Q2{i, 2};
    t = unique([t1; t2]);
    
    % find the available range
    max_id_1 = max(find(t < t1(end)));
    max_id_2 = max(find(t < t2(end)));
    
    % restrain the joint trajectory points to be within the range of imitation data     
    Q{i, 1} = [interp1(t1, Q1{i, 1}, t(1:max_id_1)); repmat(Q1{i, 1}(end, :), length(t)-max_id_1, 1)];
    Q{i, 2} = [interp1(t2, Q2{i, 1}, t(1:max_id_2)); repmat(Q2{i, 1}(end, :), length(t)-max_id_2, 1)];
    Q{i, 3} = t;
    
end

end