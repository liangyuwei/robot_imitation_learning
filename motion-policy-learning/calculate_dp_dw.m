function err = calculate_dp_dw(p_goal, R_goal, p_now, R_now)
%% This function calculates the [dp, dw] for inverse kinematics.
% p should be column vector.

p_err = p_goal - p_now;

% R_err = R_goal - R_now; % why ?

w_err = rot2omega(R_goal) - rot2omega(R_now); %R_now * rot2omega(R_err);

err = [p_err; w_err];


end