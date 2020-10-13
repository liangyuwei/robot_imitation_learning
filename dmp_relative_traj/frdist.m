function dist = frdist(curve_1, curve_2, debug)
%% This function computes Discrete Frechet Distance using the algorithm proposed in Computing discrete Fréchet distance, 1994.
% Input:
%   curve_1, curve_2 - curves with the size of DOF x N, with N being the
%   length of curve.

global ca % use the same coupling measure matrix across calls 

%% Prep
% info
[DOF1, N1] = size(curve_1);
[DOF2, N2] = size(curve_2);
assert(DOF1 == DOF2, 'Dimensions of two curves are not consistent!');

% N1 = 20;
% N2 = 20;

% initialize coupling measure
ca = ones(N1, N2) * -1;


%% Compute coupline measure (see the paper for more details)
tic;
% for i = 1 : N1
%     for j = 1 : N2
%         ca(i, j) = calculate_coupling_measure(ca, i, j, curve_1, curve_2);
%     end
% end
% dist = ca(N1, N2);
dist = calculate_coupling_measure(N1, N2, curve_1, curve_2);

if (debug)
    disp(['Time used for computing Discrete Frechet Distance for ', num2str(N1), ' x ', num2str(N1), ' path points']);
    toc;
    disp(['Frechet Distance is: ', num2str(dist)]);
end

end