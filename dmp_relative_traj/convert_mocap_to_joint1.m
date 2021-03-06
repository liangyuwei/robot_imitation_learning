function q = convert_mocap_to_joint1(ps, pe, pw, Rw, left_or_right, Re_th)
%% This function converts mocap data (positions of shoulder, elbow and wrist as well as orientation data) to 7-DOF human joint angles.
% The algorithm is computed in a similar way to the method of Dr. Hu Jin.
% Input:
%   ps - shoulder position, with the size of 3 x 1.
%   pe - elbow position
%   pw - wrist position
%   Rw - wrist rotation matrix, w.r.t world frame



%% IK
% 1 - shoulder joints
v_se_w = pe - ps; % vector pointing from shoulder to elbow; in world frame.
if (left_or_right)
    v_se_l = [-v_se_w(1), v_se_w(2), -v_se_w(3)]'; % convert to local frame of human mocap marker
else
    v_se_l = [v_se_w(1), -v_se_w(2), -v_se_w(3)]'; % convert to local frame of human mocap marker
end
% calculate rotation (euler angle (fixed frame)) of v_se_l from local frame's z-axis      
% eul = euler_from_two_vector([0, 0, 1]', v_se_l);
% gamma = eul(1); % x-axis
% beta = eul(2); % y-axis
% alpha = eul(3); % z-axis
% q1 = -beta;
% q2 = gamma;
% q3 = -alpha;

% Hujin's method (wrong...)
vec_k = unit(cross([0, 0, -1]', v_se_w)); % cross dot to yield axis of rotation     
theta = acos(dot([0, 0, -1]', v_se_w)); % by the law of cosines
r3 = angvec2r(theta,vec_k);
% pitch - y, yaw - z, roll - x
% [q(1) q(2) q(3)] = r2pry(r3); % r2pry not found...
rpy_tmp = tr2rpy(r3); % r2pry not found..
q(2) = rpy_tmp(1); % x-axis
q(1) = rpy_tmp(2); % y-axis
q(3) = rpy_tmp(3); % z-axis
q1 = q(1); q2 = q(2); q3 = q(3);


% 2 - elbow joint q4
v_ew_w = pw - pe; % vector under world frame
% q4 (from law of cosines)
tmp1 = -v_se_w;
tmp2 = v_ew_w; % should use vectors described in world frame
q4 = pi - acos(tmp1'*tmp2 / norm(tmp1) / norm(tmp2)); % this angle is calculated by the law of cosines, and is not the rotation angle of elbow joint but the rest

% 3 - elbow joint q5 + wrist joints q6, q7
if (left_or_right)
    Rs = [-1, 0, 0; ...
          0, 1, 0; ...
          0, 0, -1];
else
    Rs = [1, 0, 0; ...
          0, -1, 0; ...
          0, 0, -1];
end
% s_R_e = eul2rotm(eul, 'XYZ') * eul2rotm([q4, 0, 0], 'XYZ'); % rotation q4 around the x-axis of its own should be included!!! % since it's relative rotation, it should be post-multiplied(i.e. on the right)
s_R_e = r3 * eul2rotm([q4, 0, 0], 'XYZ');
e_R_w = s_R_e' * Rs' * Rw; % equation is Rw = Rs * s_R_e * e_R_w, try to obtain the transform from elbow frame to wrist frame...
eul1 = rotm2eul(e_R_w, 'XYZ');
gamma1 = eul1(1); % x-axis
beta1 = eul1(2); % y-axis
alpha1 = eul1(3); % z-axis

q5 = -alpha1;
q6 = gamma1;
q7 = -beta1;


%% In total
q = [q1, q2, q3, q4, q5, q6, q7]';

end