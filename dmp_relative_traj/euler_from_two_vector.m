function eul = euler_from_two_vector(A, B)
%% Calculate Euler angles (fixed axis) corresponding to rotation from A to B.
% Output: eul = [gamma, beta, alpha], as in X-Y-Z euler angle R_Z(alpha)R_Y(beta)R_X(gamma)
% Note that the rotation matrix derived from A transforming to B is specific to the frame which A and B are defined w.r.t.   

%% Angle-Axis representation
C = cross(A, B); 
if (norm(C) <= eps) % close to zero, but it might have rotated around z axis...what to do with it??   
    eul = [0, 0, 0];
    return;
end
C = C / norm(C); % normalized rotation axis

theta = acos(A' * B / norm(A) / norm(B));


%% Get the corresponding rotation matrix(uniqueness)
%{
skew_symmetric = [0, -C(3), C(2); ...
                  C(3), 0, -C(1); ...
                  -C(2), C(1), 0];
R = cos(theta) * eye(3) + (1-cos(theta)) * (C * C') + sin(theta) * skew_symmetric;
%}
R = axang2rotm([C', theta]); % equivalent functions...

%% Convert to Euler angles (X-Y-Z, fixed axis: R_Z(alpha)R_Y(beta)R_X(gamma))
%{
beta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
if (abs(beta-pi/2) <= eps)
    % gimbal lock 1, near pi/2
    alpha = 0;
    gamma = atan2(R(1,2), R(2,2));
elseif (abs(beta+pi/2) <= eps)
    % gimbal lock 2, near -pi/2
    alpha = 0;
    gamma = -atan2(R(1,2), R(2,2));
else
    alpha = atan2(R(2,1)/cos(beta), R(1,1)/cos(beta));
    gamma = atan2(R(3,2)/cos(beta), R(3,3)/cos(beta));
end

eul = [gamma, beta, alpha];
%}

eul = rotm2eul(R, 'XYZ'); % equivalent, same result

end