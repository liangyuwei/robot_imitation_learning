function [the_tot, f_tot] = optimize_joint_angles(traj_points, Steps, angular_acc_r_swing, angular_acc_l_swing)
%% This function generates a time profile of joint angles to satisfy the trajectory and motion requirements.

%% Prepare parameters
global expected_p_lx expected_p_ly expected_p_lz x_last angular_acc expected_zmp
global uLINK pos left_pivot right_pivot ref_phase


%% [1]R_up
% left foot as the pivot
% execute the right ANKLE2's trajectory
the_r_up = zeros(Steps(1), 14);
f_r_up = zeros(1, Steps(1)); % for the right ducted fan
start_point = 1; 
end_point = Steps(1);
count = 1;
x_last = [0;0;0; 0;0;0];
for n = start_point : end_point
    % set trajectory point
    expected_p_lx = traj_points(1, n); % w.r.t the world frame
    expected_p_ly = traj_points(2, n);
    expected_p_lz = traj_points(3, n);
    
    % optimize the robot's configuration
    ref_phase = 0; % left foot as the pivot
    [the, Fmin] = FindMinF_lift_3d(left_pivot); % parametric method, right as the base
    the(7) = 0; % set the pose of the left foot
    
    % record the joint configuration and thrust
    the_r_up(count, :) = the;
    f_r_up(count) = Fmin; % for the right ducted fan
    count = count + 1;
    
end

% display in demo
%{
for n = 1 : length(f_r_up) % left foot based, 
    if n ~= length(f_r_up)
        SetPose(the_r_up(n, :), [0 0 0]', 0, 1);
    else
        SetPose(the_r_up(n, :), [0 0 0]', 0, 0);
    end        
end
%}

%% [2]R_swing
% left foot as the pivot
% execute the RIGHT ANKLE2's trajectory
the_r_swing = zeros(Steps(2), 14);
f_r_swing = zeros(1, Steps(2));
start_point = Steps(1)+1; 
end_point = Steps(1)+Steps(2);
count = 1;
x_last = [the_r_up(end, 3); the_r_up(end, 4); the_r_up(end, 5); ...
          the_r_up(end, 10); the_r_up(end, 11); the_r_up(end, 12)]; % [0;0;0; 0;0;0];
for n = start_point : end_point
    % set trajectory point and angular accelerator
    expected_p_lx = traj_points(1, n);
    expected_p_ly = traj_points(2, n);
    expected_p_lz = traj_points(3, n);
    angular_acc = angular_acc_r_swing(count);
    
    % optimize the robot's joint configuration
    ref_phase = 0; % left foot as the pivot
    [the, fval] = FindMinF_swing_3d(left_pivot);
    the_r_swing(count, :) = the;
    f_r_swing(count) = fval;
    count = count + 1;

end

% display in demo
%{
for n = 1 : length(f_r_swing) % left foot based, 
    if n ~= length(f_r_swing)
        SetPose(the_r_swing(n, :), [0 0 0]', 0, 1);
    else
        SetPose(the_r_swing(n, :), [0 0 0]', 0, 0);
    end
end
%}

%% [3]R_down
% left foot as the pivot
% execute the right ANKLE2's trajectory
the_r_down = zeros(Steps(3), 14);
f_r_down = zeros(1, Steps(3)); % for the right ducted fan
start_point = Steps(1)+Steps(2)+1; 
end_point = Steps(1)+Steps(2)+Steps(3);
count = 1;
x_last = [the_r_swing(end, 3); the_r_swing(end, 4); the_r_swing(end, 5); ...
          the_r_swing(end, 10); the_r_swing(end, 11); the_r_swing(end, 12)]; % [0;0;0; 0;0;0];
for n = start_point : end_point

    % set trajectory point
    expected_p_lx = traj_points(1, n); % w.r.t the world frame
    expected_p_ly = traj_points(2, n);
    expected_p_lz = traj_points(3, n);
    
    % optimize the robot's configuration
    ref_phase = 0; % left foot as the pivot
    [the, Fmin] = FindMinF_lift_3d(left_pivot); % parametric method, right as the base
    the(7) = 90; % change the pose of the left foot
    
    % record the joint configuration and thrust
    the_r_down(count, :) = the;
    f_r_down(count) = Fmin; % for the right ducted fan
    count = count + 1;
    
end

% display in demo
%{
for n = 1 : length(f_r_down) % left foot based, 
    if n ~= length(f_r_down)
        SetPose(the_r_down(n, :), [0 0 0]', 0, 1);
    else
        SetPose(the_r_down(n, :), [0 0 0]', 0, 0);
    end        
end
%}

%% [4]L_up
% right foot as the pivot
% execute the LEFT ANKLE2's trajectory
the_l_up = zeros(Steps(4), 14);
f_l_up = zeros(1, Steps(4)); % for the right ducted fan
start_point = Steps(1)+Steps(2)+Steps(3)+1; 
end_point = Steps(1)+Steps(2)+Steps(3)+Steps(4);
count = 1;
% x_last = [the_l_up(end, 3); the_l_up(end, 4); the_l_up(end, 5); ...
%           the_l_up(end, 10); the_l_up(end, 11); the_l_up(end, 12)]; % [0;0;0; 0;0;0];
x_last = [90; 0; -90; 90; 0; -90];
for n = start_point : end_point
    
   
    % set trajectory point
    expected_p_lx = traj_points(1, n); % w.r.t the world frame
    expected_p_ly = traj_points(2, n);
    expected_p_lz = traj_points(3, n);
    
    % optimize the robot's configuration
    ref_phase = 1; % right foot as the pivot
    [the, Fmin] = FindMinF_lift_3d(right_pivot); % parametric method, right as the base
    
    % record the joint configuration and thrust
    the_l_up(count, :) = the;
    f_l_up(count) = Fmin; % for the right ducted fan
    count = count + 1;
end

% display in demo
%{
for n = 1 : length(f_l_up) % left foot based, 
    if n ~= length(f_l_up)
        SetPose(the_l_up(n, :), [0 0 0]', 1, 1);
    else
        SetPose(the_l_up(n, :), [0 0 0]', 1, 0);
    end        
end
%}

%% [5]L_swing
% rightt foot as the pivot
% execute the LEFT ANKLE2's trajectory
the_l_swing = zeros(Steps(5), 14);
f_l_swing = zeros(1, Steps(5));
start_point = Steps(1)+Steps(2)+Steps(3)+Steps(4)+1; 
end_point = Steps(1)+Steps(2)+Steps(3)+Steps(4)+Steps(5);
count = 1;
x_last = [the_l_up(end, 3); the_l_up(end, 4); the_l_up(end, 5); ...
          the_l_up(end, 10); the_l_up(end, 11); the_l_up(end, 12)]; % [0;0;0; 0;0;0];
for n = start_point : end_point
    
     % display the message temporarily
    ['Currently updating Step ', int2str(count), '...']
    
    
    % set trajectory point and angular accelerator
    expected_p_lx = traj_points(1, n);
    expected_p_ly = traj_points(2, n);
    expected_p_lz = traj_points(3, n);
    angular_acc = angular_acc_l_swing(count);
    
    % optimize the robot's joint configuration
    ref_phase = 1; % right foot as the pivot
    [the, fval] = FindMinF_swing_3d(right_pivot);
    the_l_swing(count, :) = the;
    f_l_swing(count) = fval;
    count = count + 1;

end

% display in demo
%{
for n = 1 : length(f_l_swing) % left foot based, 
    if n ~= length(f_l_swing)
        SetPose(the_l_swing(n, :), [0 0 0]', 1, 1);
    else
        SetPose(the_l_swing(n, :), [0 0 0]', 1, 0);
    end        
end
%}


%% [6]L_down
% right foot as the pivot
% execute the LEFT ANKLE2's trajectory
the_l_down = zeros(Steps(6), 14);
f_l_down = zeros(1, Steps(6)); % for the right ducted fan
start_point = Steps(1)+Steps(2)+Steps(3)+Steps(4)+Steps(5)+1; 
end_point = Steps(1)+Steps(2)+Steps(3)+Steps(4)+Steps(5)+Steps(6);
count = 1;
x_last = [the_l_swing(end, 3); the_l_swing(end, 4); the_l_swing(end, 5); ...
          the_l_swing(end, 10); the_l_swing(end, 11); the_l_swing(end, 12)]; % [0;0;0; 0;0;0];
for n = start_point : end_point
    
    % display the message temporarily
    ['Currently updating Step ', int2str(count), '...']
    
    % set trajectory point
    expected_p_lx = traj_points(1, n); % w.r.t the world frame
    expected_p_ly = traj_points(2, n);
    expected_p_lz = traj_points(3, n);
    
    % optimize the robot's configuration
    ref_phase = 1; % right foot as the pivot
    [the, Fmin] = FindMinF_lift_3d(right_pivot); % parametric method, right as the base
    the(14) = 90;
    
    % record the joint configuration and thrust
    the_l_down(count, :) = the;
    f_l_down(count) = Fmin; % for the right ducted fan
    count = count + 1;
end

% display in demo
%{
for n = 1 : length(f_l_down) % left foot based, 
    if n ~= length(f_l_down)
        SetPose(the_l_down(n, :), [0 0 0]', 1, 1);
    else
        SetPose(the_l_down(n, :), [0 0 0]', 1, 0);
    end        
end
%}


%% Add transition phases before/after the edging phases
% transition 1, added befor R_up
the_ini = [90, 0, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, 0, 0];
the_end = the_r_up(1, :);
steps_trans_1 = 20;
the_trans_1 = zeros(steps_trans_1, 14);
f_trans_1 = zeros(1, steps_trans_1);
for n = 1:size(the_trans_1, 2)
    if the_ini(n) == the_end(n)
        the_trans_1(:, n) = ones(steps_trans_1, 1) .* the_ini(n);
    else
        the_trans_1(:, n) = the_ini(n) : (the_end(n) - the_ini(n))/(steps_trans_1 - 1) : the_end(n);
    end
end

% display the results
%{
for n = 1:steps_trans_1
    if n ~= steps_trans_1
        SetPose(the_trans_1(n, :), [0 0 0]', 0, 1);
    else
        SetPose(the_trans_1(n ,:), [0 0 0]', 0, 0);
    end
end
%}

% transition 2, added after the L_down phase
the_ini = the_l_down(end, :);
the_end = [90, 0, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, 0, 90]; 
steps_trans_2 = 20;
the_trans_2 = zeros(steps_trans_2, 14);
f_trans_2 = zeros(1, steps_trans_2);
for n = 1:size(the_trans_2, 2)
    if the_ini(n) == the_end(n)
        the_trans_2(:, n) = ones(steps_trans_2, 1) .* the_ini(n);
    else
        the_trans_2(:, n) = the_ini(n) : (the_end(n) - the_ini(n))/(steps_trans_2 - 1) : the_end(n);
    end
end

% display the results
%{
for n = 1:steps_trans_2
    if n ~= steps_trans_2
        SetPose(the_trans_2(n, :), [0 0 0]', 1, 1);
    else
        SetPose(the_trans_2(n ,:), [0 0 0]', 1, 0);
    end
end
%}


%% Return the results
% note that the_x contains 14 columns now!!! This is for convenience in using wholebody.m to update the robot's pose  
% the_swing now contains 14 columns, two of which should be deleted.
the_tot = [the_trans_1; the_r_up; the_r_swing; the_r_down; the_l_up; the_l_swing; the_l_down; the_trans_2];
f_tot = [f_trans_1, f_r_up, f_r_swing, f_r_down, f_l_up, f_l_swing, f_l_down, f_trans_2];


%% Demonstrate the results
%
% set the reference sign
ref = ones(1, length(f_tot));
ref(1:(steps_trans_1+Steps(1)+Steps(2)+Steps(3))) = 0;
% display the whole process
for n = 1:length(f_tot)
    if n ~= length(f_tot)
        SetPose(the_tot(n, :), [0, 0, 0]', ref(n), 1);
    else
        SetPose(the_tot(n, :), [0, 0, 0]', ref(n), 0);
    end
end
% display the needed thrust
figure;
bar(f_tot);
grid on;
%}


end
