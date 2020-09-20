function dtw_mat = calculate_dtw_cost(n, m, curve_1, curve_2)
%% This function computes DTW distance matrix using the algorithm proposed in paper.
% Note that input curves have the size of DOF x N, with N being the length.

global dtw_mat

%% Using Euclidean distance
d = @(x, y) norm(x-y);

%% Computation
for i = 2 : n+1
    for j = 2 : m+1
        cost = d(curve_1(:, n), curve_2(:, m));
        dtw_mat = cost + min([dtw_mat(i-1, j, curve_1, curve_2), ...
                              dtw_mat(i, j-1, curve_1, curve_2), ...
                              dtw_mat(i-1, j-1, curve_1, curve_2)]);
    end
end

end