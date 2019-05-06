function err = calculate_dp_dw(p_goal, R_goal, p_now, R_now)
%% This function calculates the [dp, dw] for inverse kinematics.
% p should be column vector.

p_err = p_goal - p_now;

R_err = R_now' * R_goal; %R_goal - R_now; %R_now - R_goal; % why ?

w_err = R_now * rot2omega(R_err); % equal to [0,0,0] when R_now = R_goal; is R_now really needed??? R_now is current pose in the world frame!!!
% w_err = rot2omega(R_goal) - rot2omega(R_now); 
% w_err = R_now * rot2omega(R_err); % transform to current frame

err = [p_err; w_err];


end