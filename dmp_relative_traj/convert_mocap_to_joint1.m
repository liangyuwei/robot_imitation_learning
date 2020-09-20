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
eul = euler_from_two_vector([0, 0, 1]', v_se_l);
gamma = eul(1); % x-axis
beta = eul(2); % y-axis
alpha = eul(3); % z-axis
q1 = -beta;
q2 = gamma;
q3 = -alpha;

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
s_R_e = eul2rotm(eul, 'XYZ') * eul2rotm([q4, 0, 0], 'XYZ'); % rotation q4 around the x-axis of its own should be included!!! % since it's relative rotation, it should be post-multiplied(i.e. on the right)
e_R_w = s_R_e' * Rs' * Rw; % equation is Rw = Rs * s_R_e * e_R_w, try to obtain the transform from elbow frame to wrist frame...
eul1 = rotm2eul(e_R_w, 'XYZ');
gamma1 = eul1(1); % x-axis
beta1 = eul1(2); % y-axis
alpha1 = eul1(3); % z-axis

q5 = -alpha1;
q6 = gamma1;
q7 = -beta1;



% q5
%{
elb_z = v_ew_w;
elb_x = cross(v_se_w, v_ew_w);
elb_y = cross(elb_z, elb_x);
Re = [elb_x, elb_y, elb_z]; % orientaion w.r.t world, just like Rw
e_R_w = Re' * Rw; % wrist orientation under elbow frame % already have Re, Rw in world frame, and the relation Rw = Re * e_R_w by chain rule
tmp_x = e_R_w(:, 1); tmp_y = e_R_w(:, 2); tmp_z = e_R_w(:, 3); 
tmp1 = [tmp_y(1), tmp_y(2), 0.0]';
tmp2 = [0, 1, 0]'; %[0.0, tmp_y(2), 0.0]'; % the y-axis of elbow frame w.r.t elbow frame
q5 = sign(tmp_y(1)) * acos(tmp1'*tmp2/norm(tmp1)/norm(tmp2)); % use Re or just ps & pe & pw 

% q6
tmp1 = [0.0, tmp_z(2), tmp_z(3)]';
tmp2 = [0, 0, 1]'; % the z axis of elbow frame w.r.t elbow frame
q6 = -sign(tmp_z(2)) * acos(tmp1'*tmp2/norm(tmp1)/norm(tmp2));
% q7 
tmp1 = [tmp_z(1), 0.0, tmp_z(3)]';
tmp2 = [0, 0, 1]'; % the z axis of elbow frame w.r.t elbow frame
q7 = -sign(tmp_z(1)) * acos(tmp1'*tmp2/norm(tmp1)/norm(tmp2));
%}


%% In total
q = [q1, q2, q3, q4, q5, q6, q7]';

end