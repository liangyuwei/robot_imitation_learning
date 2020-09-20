function q = convert_mocap_to_joint(ps, pe, pw, Rw, left_or_right)
%% This function converts mocap data (positions of shoulder, elbow and wrist as well as orientation data) to 7-DOF human joint angles.
% The algorithm is computed in a similar way to the method of Dr. Hu Jin.
% Input:
%   ps - shoulder position, with the size of 3 x 1.
%   pe - elbow position
%   pw - wrist position
%   Rw - wrist rotation matrix, w.r.t world frame


%% IK
% shoulder joints
v_se_w = pe - ps; % vector pointing from shoulder to elbow; in world frame.
if (left_or_right)
    v_se_l = [-v_se_w(1), v_se_w(2), -v_se_w(3)]'; % convert to local frame of human mocap marker
else
    v_se_l = [v_se_w(1), -v_se_w(2), -v_se_w(3)]'; % convert to local frame of human mocap marker
end
% q1
tmp1 = [v_se_l(1), 0.0, v_se_l(3)]';
tmp2 = [0, 0, 1]'; %[0.0, 0.0, v_se_l(3)]'; % the same 
q1 = -sign(v_se_l(1)) * acos(tmp1'*tmp2 / norm(tmp1) / norm(tmp2));
% q2
tmp1 = [0.0, v_se_l(2), v_se_l(3)]';
tmp2 = [0, 0, 1]'; %[0.0, 0.0, v_se_l(3)]'; % the same
q2 = -sign(v_se_l(2)) * acos(tmp1'*tmp2 / norm(tmp1) / norm(tmp2)); % in radius
% q3
tmp1 = [v_se_l(1), v_se_l(2), 0.0]';
tmp2 = [0, 1, 0]'; %[0.0, v_se_l(2), 0.0]'; % the same
q3 = sign(v_se_l(1)) * acos(tmp1'*tmp2 / norm(tmp1) / norm(tmp2));

% elbow joints
v_ew_w = pw - pe; % vector under world frame
if (left_or_right)
    v_ew_l = [-v_ew_w(1), v_ew_w(2), -v_ew_w(3)]'; % convert to local frame of human mocap marker
else
    v_ew_l = [v_ew_w(1), -v_ew_w(2), -v_ew_w(3)]'; % convert to local frame
end
% q4
tmp1 = -v_se_w;
tmp2 = v_ew_w; % should use vectors described in world frame
q4 = pi - acos(tmp1'*tmp2 / norm(tmp1) / norm(tmp2)); % this angle is calculated by the law of cosines, and is not the rotation angle of elbow joint but the rest
% q5
elb_z = v_ew_w;
elb_x = cross(v_se_w, v_ew_w);
elb_y = cross(elb_z, elb_x);
Re = [elb_x, elb_y, elb_z]; % orientaion w.r.t world, just like Rw
e_R_w = Re' * Rw; % wrist orientation under elbow frame % already have Re, Rw in world frame, and the relation Rw = Re * e_R_w by chain rule
tmp_x = e_R_w(:, 1); tmp_y = e_R_w(:, 2); tmp_z = e_R_w(:, 3); 
tmp1 = [tmp_y(1), tmp_y(2), 0.0]';
tmp2 = [0, 1, 0]'; %[0.0, tmp_y(2), 0.0]'; % the y-axis of elbow frame w.r.t elbow frame
q5 = sign(tmp_y(1)) * acos(tmp1'*tmp2/norm(tmp1)/norm(tmp2)); % use Re or just ps & pe & pw 

% wrist joints
% q6
tmp1 = [0.0, tmp_z(2), tmp_z(3)]';
tmp2 = [0, 0, 1]'; % the z axis of elbow frame w.r.t elbow frame
q6 = -sign(tmp_z(2)) * acos(tmp1'*tmp2/norm(tmp1)/norm(tmp2));
% q7 
tmp1 = [tmp_z(1), 0.0, tmp_z(3)]';
tmp2 = [0, 0, 1]'; % the z axis of elbow frame w.r.t elbow frame
q7 = -sign(tmp_z(1)) * acos(tmp1'*tmp2/norm(tmp1)/norm(tmp2));


%% In total
q = [q1, q2, q3, q4, q5, q6, q7]';

end