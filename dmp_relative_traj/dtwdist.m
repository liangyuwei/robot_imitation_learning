function dist = dtwdist(curve_1, curve_2)
%% This function computes DTW measure using the algorithm on wiki pedia.
% Input:
%   curve_1, curve_2 - curves with the size of DOF x N, with N being the
%   length of curve.

% global dtw_mat % use the same coupling measure matrix across calls 

%% Prep
% info
[DOF1, N1] = size(curve_1);
[DOF2, N2] = size(curve_2);
assert(DOF1 == DOF2, 'Dimensions of two curves are not consistent!');

N1 = 5;
N2 = 5;

% initialize coupling measure
dtw_mat = ones(N1+1, N2+1) * Inf;
dtw_mat(1, 1) = 0;

%% Using Euclidean distance
d = @(x, y) norm(x-y);

%% Compute DTW measure
tic;
for i = 1 : N1
    for j = 1 : N2
        cost = d(curve_1(:, i), curve_2(:, j));
        dtw_mat(i+1, j+1) = cost + min([dtw_mat(i, j+1), ...
                              dtw_mat(i+1, j), ...
                              dtw_mat(i, j)]);
    end
end
dist = dtw_mat(end, end);
disp(['Time used for computing DTW distance matrix for ', num2str(N1), ' x ', num2str(N1), ' path points']);
toc;

disp(['DTW Distance is: ', num2str(dist)]);

end