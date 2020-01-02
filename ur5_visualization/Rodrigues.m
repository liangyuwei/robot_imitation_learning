function R = Rodrigues(w,dt)
%% Computation of Rodriguez transform
% w - rotation axis, should be column vector of size 3
% dt - rotation angle

% Normalization
th = norm(w)*dt;
wn = w/norm(w);
w_wedge = [0 -wn(3) wn(2);wn(3) 0 -wn(1);-wn(2) wn(1) 0]; % skew-symmetric matrix for wn

% Rodriguez formula, correct, a bit transformation from that of BaiDu Baike
R = eye(3) + w_wedge * sin(th) + w_wedge^2 * (1-cos(th));

end
