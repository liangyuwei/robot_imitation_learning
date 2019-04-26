function loss = gen_joint_traj_loss_stepbystep(q)
%% This function computes the cost function for generating the joint trajectory

global uLINK

% expected wrist position
global cur_exp_xw

% the current wrist position
global th
th = q;
update_robot_joint;
ForwardKinematics(2);
cur_xw = uLINK(7).p;

% loss function
loss = sum(((cur_xw - cur_exp_xw)*10) .^ 2);

end