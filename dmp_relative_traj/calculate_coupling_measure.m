function ca_ij = calculate_coupling_measure(i, j, curve_1, curve_2)
%% This function computes coupling measure using the algorithm proposed in paper.
% Note that input curves have the size of DOF x N, with N being the length.

global ca % use the same

%% Using Euclidean distance
d = @(x, y) norm(x-y);

% disp(ca);

%% Computation
if ca(i, j) > -1
    ca_ij = ca(i, j);
elseif i == 1 && j == 1
    ca_ij = d(curve_1(:, 1), curve_2(:, 1));
elseif i > 1 && j == 1
    ca_ij = max(calculate_coupling_measure(i-1, 1, curve_1, curve_2), d(curve_1(:, i), curve_2(:, 1)));
elseif i == 1 && j > 1
    ca_ij = max(calculate_coupling_measure(1, j-1, curve_1, curve_2), d(curve_1(:, 1), curve_2(:, j)));
elseif i > 1 && j > 1
    ca_ij = max(min([calculate_coupling_measure(i-1, j, curve_1, curve_2), ...
                    calculate_coupling_measure(i-1, j-1, curve_1, curve_2), ...
                    calculate_coupling_measure(i, j-1, curve_1, curve_2)]), ...
                d(curve_1(:, i), curve_2(:, j)));
else
    ca_ij = Inf;
end

% store the result
ca(i, j) = ca_ij;

end
