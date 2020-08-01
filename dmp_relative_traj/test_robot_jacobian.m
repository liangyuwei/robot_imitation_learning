




J = [0.22971   0.314549   0.164367  0.0704364 -0.0799635          0          0
  0.361718   -0.12812 -0.0187072   0.166767  0.0797464          0          0
  -0.13446   0.185612  0.0556225 0.00151637 -0.0128483          0          0
  0.813754  -0.577403   0.123807   0.764152   0.633043          0          0
 -0.342122  -0.568239  -0.770695  -0.327801   0.546421          0          0
  0.469848   0.586267  -0.625061   0.555534  -0.548344          0          0];

dx = [0.300966 -0.212813  0.929586         0         0         0]';

A = J*J';



[U, S, V] = svd(A); % A = USV';

%% this is actually what pinv() is based on .... check out help pinv....

% pinv of A = 
S_inv = zeros(size(S));
threshold = 0.01;
for i = 1:size(S, 1)
    if S(i, i) >= threshold
        S_inv(i, i) = 1 / S(i, i);
    end
end

A_inv = V*S_inv*U';

A*A_inv



%% dx = J * dq

% what if only use position data?
J = J(1:3, :);
dx = dx(1:3);

% check out the performance
dq = J' * pinv(J*J') * dx;

J * dq

dx



%%

J = [0.0124851    0.0805109   -0.0302856   -0.0101402
  0.00718802   -0.0397021    0.0149347   0.00500056
  1.1057e-06  0.000475987  -0.00078304  -0.00372191
 9.00581e-05     0.442275    -0.442275    -0.442275
-2.59882e-06     0.896879    -0.896879    -0.896879
          -1  3.74996e-05 -3.74996e-05 -3.74996e-05];
      
dx = [-0.834886 -0.416948 -0.359332         0         0         0]';


[U, S, V] = svd(J*J'); % J = U*S*V'

T = zeros(size(S));
threshold = max(size(J)) * eps(norm(J));
for i = 1 : rank(S)
    if S(i, i) > threshold
        T(i, i) = 1/S(i, i);
    end
end
JJ_inv = V * T * U';

dq = J' * JJ_inv * dx;


%% fast version, set tolerance, and display result
[U, S, V] = svd(J);
dq = pinv(J, 0.001) * dx

J * dq

dx


