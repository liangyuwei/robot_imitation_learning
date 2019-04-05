function [the,position,ref,the_bound,position_bound,ref_bound, f]=Opt_H(Pgoal,priority)
%% This function is for ...
% Pgoal - the target position on the other side of the ditch;
% priority - step higher or lower(???)

%% Set up parameters for ...
global uLINK F P_goal Pend pos expected_zmp 
th = []; % the joint motion in joint space

ToRad = pi/180;
ToDeg = 180/pi;

% for recording the thrust
f = [];


% front leg process
the_for = []; % joint motion for the forward leg
position_for = []; % end joint position(???)
ref_for = []; % counteract the offset for display(???)

bound = []; % record boundries

% transition process
the_mid = [];
position_mid = [];
ref_mid = [];

% latter leg process
the_inv = [];
position_inv = [];
ref_inv = [];

% 
p_b = [];
the_b = [];
ref_b = [];

%% Calculate corresponding joint configurations with different end-point position
iterator = -250:5:250;
th = zeros(length(iterator), 7);

for k = 1:length(iterator) % height of what???
    q = Optimize(iterator(k)); % draw boundries???
    th(k,:) = [q, iterator(k)];
end


%% Fit a second-order polynomial to the boundary(f_y and f_h) respectively
% Prepare data
joint_feet_displacement = [0, 0, 40]';
for i = 1:length(th(:,1))
    % End-state joint configuration(the boundry)
    the0 = [ 90, 0, th(i,3), -th(i,2), -th(i,1), 0, ...
            -90, 0, th(i,4),  th(i,5), -th(i,6), 0];
    write(the0); % apply new joint configuration and update robot's joint configuration
    
    % 'p1' is the vector pointing to the base from the left foot in the display plane(can be verified using vector arithmetic)  
    p1 = ([0, 0, 0] - uLINK(7).p') + (pos' + joint_feet_displacement');
    p_b = [p_b; p1]; % array of 'p1' under different joint configuration
    the_b = [the_b; the0]; % record corresponding joint configuration(end state)
    ref_b = [ref_b; 0]; % ???
    
    pr = uLINK(13).p' - uLINK(7).p'; % right ankle position w.r.t the left ankle coordinate
    bound = [bound; pr]; % record right ankle's position as a point on the boundry
end

% Fit second-order polynomials for the reachable boundary
f_h = polyfit(bound(:,3),bound(:,2),2);
f_y = polyfit(bound(:,2),bound(:,3),2);

%% Set approriate Pend according to the end position goal(Pgoal)(using boundary function)
Pend = [0, 920, 0];

%% (Right Leg Version)Find minimum required forces for some discrete points on the trajectory
% Since 'bound' is calculated with the use of right&left ankles, generating
% the boundary function 'f_y' and 'f_h', with which Pend is defined, thus,
% Pend is the target position of the right ankle w.r.t the left ankle
% coordinate.
%{\
[th, ~, ~] = FindminF_traj(Pend); % Pend = [0, 920, 0]

for i=1:length(th(:,1)) 
    % note that what th contains are the joint configurations for EACH
    % POINT on the trajectory, which has the minimum requirements for force
    the0=[90 0 th(i,3) -th(i,4) -th(i,5) 0 0 -90 0 th(i,9) th(i,10) -th(i,11) 0 0];
    
    % apply to the robot's joint configuration, and update the robot's
    % position and pose
    write(the0);
    % After updating the robot's joint configuration and calculating p and
    % R for each joint,
    % record the location of the base w.r.t the world frame
    p1 = [0 0 0] - uLINK(7).p' + pos' + joint_feet_displacement';
    position_for = [position_for;p1];
    
    % record motor's angles
    the_for = [the_for;the0];
    ref_for = [ref_for;0];
    
end
%}


%% Swing phase
% trajectory parameters (of the left ducted fan)(This should've been set before the transition phase and used to plan the transition state)
global traj_height traj_radius

% display the message
for i = 1:5
    'Now starting optimization for Swing phase...'
end

% Initialize variables for storage
ref_swing = [];
position_swing = [];
position_left_ducted_fan_swing = [];
position_left_ankle_swing = [];
position_com_swing = [];
position_main_body_com = [];
the_swing = [];
f = [];
traj_x = [];
traj_y = [];
traj_z = [];
traj_angular_acc = [];
traj_angular_vel = [];

% kinematic parameters, in radius (linear angular acceleration)
%{
angular_acc_z = @(t) -0.00075*pi*t + 0.0075 * pi;
angular_vel_z = @(t) -0.000375 * pi * t .^2 + 0.0075 * pi * t;
angle_z = @(t) -0.000125 * pi * t .^3 + 0.00375 * pi * t .^2;
ToDeg = 180/pi;
%}

% kinematic parameters, in radius (sinusoidal angular acceleration)
angular_acc_z = @(t) pi^2 / 400 * sin(pi/10*t); 
angular_vel_z = @(t) pi/40 * (1 - cos(pi/10*t)); 
angle_z = @(t) pi/40 * (t - 10/pi*sin(pi/10*t));
ToDeg = 180/pi;

% display planned kinematic parameters, angular accelerator/angular velocity/angle displacement 
%{
% angular acceleration
figure;
plot(0:0.1:20, angular_acc_z(0:0.1:20)*ToDeg, 'b-'); grid on;
xlabel('Time/s'); ylabel('angular acceleration/(deg * s^-2)');
title('Planned angular acceleration');
% angular velocity
figure;
plot(0:0.1:20, angular_vel_z(0:0.1:20)*ToDeg, 'r-'); grid on;
xlabel('Time/s'); ylabel('angular velocity/(deg * s^-1)');
title('Planned angular velocity');
% angle displacement
figure;
plot(0:0.1:20, angle_z(0:0.1:20)*ToDeg, 'g-'); grid on;
xlabel('Time/s'); ylabel('angle/deg');
title('Planned angle trajectory');
%}

% Generate a trajectory with the specification of angular velocities and angular acceleration(linear angular acceleration) 
swing_steps = 50;
T_period = 20;
for n = 0:swing_steps
    % current time
    t = T_period / swing_steps * n;
    
    % current kinematic parameters
    dw_z = angular_acc_z(t);
    w_z = angular_vel_z(t);
    fi_angle = angle_z(t);
    
    % add current trajectory point
    traj_x = [traj_x; - traj_radius * sin(fi_angle) + expected_zmp(1)];
    traj_y = [traj_y; - traj_radius * cos(fi_angle) + expected_zmp(2)];
    traj_z = [traj_z; traj_height + expected_zmp(3)];
    traj_angular_acc = [traj_angular_acc; dw_z];
    traj_angular_vel = [traj_angular_vel; w_z];
    
end


% display the planned trajectory point at each step
%{
figure;
plot3(traj_x, traj_y, traj_z, 'b.');
grid on; hold on;
plot3(0, 940, 0, 'rx');
axis([-600, 0, 0, 1000, 0, 350]);
title('Planned trajectory at each step');
xlabel('x'); ylabel('Length/mm'); zlabel('Height/mm');
view(65, 55); % 3d
% view(90, 90); % x-y
% view(90, 0); % y-z
%}

% prepare required global variables
global expected_p_lx expected_p_ly expected_p_lz x_last angular_acc mu_k avg_r

% average of distance of the sole to the ZMP; for more details, check out my weekly report at 20/04/2018  
lx = 40; ly = 220;
avg_r = integral2(@(x,y)(x .^ 2 + y .^ 2) .^ 0.5, -lx/2, lx/2, -ly/2, ly/2) / (lx * ly); 

% apply optimization
x_last = [ 90;0;-90;  51.78; -81.78; 30];%  90;0;-90 ]; % ['left leg', 'right leg']
for i = 1:length(traj_x)
    
    message = ['Optimization for Step ', int2str(i), '...']
    
    % trajectory of the left ankle
    expected_p_lx = traj_x(i); 
    expected_p_ly = traj_y(i);
    expected_p_lz = traj_z(i); % w.r.t the world frame
    
    % update the angular accelerator
    angular_acc = traj_angular_acc(i);
    
    % Run optimization
    [q, fval] = FindMinF_based_on_ZMP;
    
    % record the joint configuration
    the_swing = [the_swing; q];    
    
    % update the robot's state for calculating some values
    write(q);  
    f = [f, fval];
    position_swing = [position_swing; uLINK(1).p']; % unused for swing phase
    ref_swing = [ref_swing; 1];
    
    % record the left ducted fan
    position_left_ducted_fan_swing = [position_left_ducted_fan_swing; uLINK(16).p'];
    position_left_ankle_swing = [position_left_ankle_swing; uLINK(7).p'];
    
    % record the com
    mc = 0; M =0;
    for j = 1:13
        mc = mc + uLINK(j).c * uLINK(j).m;
        M = M + uLINK(j).m;
    end
    com = mc./M;
    position_com_swing = [position_com_swing; com'];
    
    % record the CoM of the main body
    mc = 0; m_G =0;
    for k = 1:15
        if k ~= 6 && k ~= 7 && k~= 8 % eliminating the left foot's weight
            mc = mc + uLINK(k).c * uLINK(k).m;
            m_G = m_G + uLINK(k).m;
        end
    end
    p_G = mc./m_G;
    position_main_body_com = [position_main_body_com; p_G'];

end

% display the robot's motion with the changes in thrust and theta simultaneously
%{
q_ankle_plot = zeros(size(the_swing(:, 6)));
f_plot = zeros(size(f));
figure; % Contain the two views within one figure
for iter = 1:size(the_swing, 1)
    %% Display the robot's movement in real-time
    subplot(1,3,1);
    if iter ~= size(the_swing, 1)
        SetPose(the_swing(iter, :), [0,0,0]', 1, 1);
    else
        SetPose(the_swing(iter, :), [0,0,0]', 1, 0);
    end
    
    %% Display the change in thrust and theta
    % the robot's state is already updated
    
    % calculate the necessary variables
    q_ankle_plot(iter) = the_swing(iter, 6);
    f_plot(iter) = f(iter);
    
    % display thrust at each step
    subplot(1,3,2); % switch to the other view
    bar(q_ankle_plot);
    title('Rotation angle of the left ankle');
    xlabel('Steps'); ylabel('Angle/deg');
    grid on;
    
    % display rotational angles of the ankle at each step
    subplot(1,3,3);
    bar(f_plot);
    title('Needed thrust from the left ducted fan');
    xlabel('Steps'); ylabel('Force/N');
    grid on;
    
    % switch the first sub-window for the display of the robot's configuration 
    subplot(1,3,1);
    
end
%}

% display the robot's motion with the changes in angular acceleration and velocity simultaneously
%{
angular_vel_plot = zeros(size(traj_angular_vel));
angular_acc_plot = zeros(size(traj_angular_acc));
figure; % Contain the two views within one figure
for iter = 1:size(the_swing, 1)
    %% Display the robot's movement in real-time
    subplot(1,3,1);
    if iter ~= size(the_swing, 1)
        SetPose(the_swing(iter, :), [0,0,0]', 1, 1);
    else
        SetPose(the_swing(iter, :), [0,0,0]', 1, 0);
    end
    
    %% Display the change in angular acceleration and velocity
    % the robot's state is already updated
    
    % calculate the necessary variables
    angular_acc_plot(iter) = traj_angular_acc(iter);
    angular_vel_plot(iter) = traj_angular_vel(iter);
    
    % display angular acceleration at each step
    subplot(1,3,2); % switch to the other view
    bar(angular_acc_plot * ToDeg);
    title('Angular accelerator at each step');
    xlabel('Steps'); ylabel('Angular accelerator/deg*s^(-2)');
    grid on;
    
    % display angular velocity at each step
    subplot(1,3,3);
    bar(angular_vel_plot * ToDeg);
    title('Angular velocity at each step');
    xlabel('Steps'); ylabel('Angular velocity/deg*s^(-1)');
    grid on;
    
    % switch the first sub-window for the display of the robot's configuration 
    subplot(1,3,1);
    
end
%}

% save the results for comparison
%{
err = sqrt( sum( (position_left_ankle_swing - [traj_x', traj_y', traj_z']).^2, 2) );
save('simulations_results_traj_planning_swing_phase_x.mat', ...
     'mu_k', 'angular_acc', 'traj_radius', 'traj_height', 'the_swing', 'f', 'position_left_ankle_swing', ...
     'position_left_ducted_fan_swing', 'position_main_body_com', 'err');
%}

% display the executed trajectory as well as that of planned ahead(for improved version of optimization)
%{
figure;
% axis([-400,400,-200,1100,0,150]);
xlabel('x/mm'); ylabel('y/mm'); zlabel('z/mm');
title('Blue - planned; Red - executed');
plot3(position_left_ankle_swing(:, 1), position_left_ankle_swing(:, 2), position_left_ankle_swing(:, 3), 'rx');
hold on; grid on;
plot3(traj_x, traj_y, traj_z, 'b.');
legend('Planned', 'Executed');
xlabel('x');
ylabel('Length/mm');
zlabel('Height/mm');
title('Trajectory of the left ducted fan');
axis([-960, 20, -20, 960, 0, 350]);
view(60, 35);
%}

% display the errors
%{
err = sqrt( sum( (position_left_ankle_swing - [traj_x, traj_y, traj_z]).^2, 2) );
figure; 
plot(1:length(err), err, 'r.');
grid on;
xlabel('Step'); ylabel('Error');
title('Error at each step');
axis([0, length(err), 0, max(err)]);
%}

%% Validation of the optimization results for the swing phase
%{
% call function to iterate through the whole joint space
record_feasible_thrust;

% calculate the CoM of the main body
q_swing_op = the_swing_optimized;
q_swing_op(6) = 0;
q_swing_op(end) = 0;
write(q_swing_op);
p_zmp = expected_zmp;
mc = 0; m_G =0; m_l = 0;
for i = 1:15
    if i ~= 6 && i ~= 7 && i~= 8 % eliminating the left foot's weight
        mc = mc + uLINK(i).c * uLINK(i).m;
        m_G = m_G + uLINK(i).m;
    else
        m_l = m_l + uLINK(i).m;
    end
end
p_G = mc./m_G;
r_G = p_G' - p_zmp;

% save the optimized result
savefile = 'record_feasible_thrust';
the_swing_optimized = the_swing(end, :);
f_optimized = f(end);
dist_r_G_optimized = abs(r_G(2));
save(savefile, 'the_swing_optimized', 'f_optimized', 'dist_r_G_optimized', '-append');


% check out the results
%{
load('record_feasible_thrust.mat');
for m = 1:length(f_stg)
    if m ~= length(f_stg)
        SetPose(the_stg(m, :), [0, 0, 0]', 1, 1);
    else
        SetPose(the_stg(m, :), [0, 0, 0]', 1, 0);        
    end
end
%}

% plot all the thrusts, according to the distance from the main body's CoM to the rotational axis
load('record_feasible_thrust.mat');
[sorted_dist_r_G_stg, ascend_index] = sort(dist_r_G_stg);
figure;
plot(sorted_dist_r_G_stg, f_stg(ascend_index), 'b.');
hold on;
plot(dist_r_G_optimized, f_optimized, 'rx');
axis([0, max(dist_r_G_stg)+10, 0, max(f_stg)+10]);
grid on;
title(['angular acc = ', int2str(angular_acc*180/pi), ...
        ', mu_k = ', num2str(mu_k), ', traj radius = ', num2str(traj_radius), ...
        ', traj height = ', num2str(traj_height), '.']);
xlabel('Distance from the main body''s CoM to the rotational axis'); ylabel('Needed thrust/N');
% find(f_stg<f_optimized)

% plot all the stored results
%{
for result_index = 1:6
    % load the storage file
    load(['record_feasible_thrusts/record_feasible_thrust', int2str(result_index), '.mat']);
 
    % display
    [sorted_dist_r_G_stg, ascend_index] = sort(dist_r_G_stg);
    figure;
    plot(sorted_dist_r_G_stg, f_stg(ascend_index), 'b.');
    hold on;
    plot(dist_r_G_optimized, f_optimized, 'rx');
    axis([0, max(dist_r_G_stg)+10, 0, max(f_stg)+10]);
    grid on;
    title(['angular acc = ', int2str(angular_acc*180/pi), ...
        ', mu_k = ', num2str(mu_k), ', traj radius = ', num2str(traj_radius), ...
        ', traj height = ', num2str(traj_height), '.']);
    xlabel('Distance from the main body''s CoM to the rotational axis'); ylabel('Needed thrust/N');
    
end
%}

% check out the results - Exp 3 & 4, why r_G is close to 0?
%{
% optimized
the_swing_optimized(6) = 0; % set the foot horizontal
the_swing_optimized(end) = 0; % set the rotational angle to 0
SetPose(the_swing_optimized, [0, 0, 0]', 1, 0);
hold on;
% through iteration search, exp3
SetPose(the_stg(235, :), [0, 0, 0]', 1, 0);
hold on;

p_zmp = expected_zmp;
mc = 0; m_G =0; m_l = 0;
for i = 1:15
    if i ~= 6 && i ~= 7 && i~= 8 % eliminating the left foot's weight
        mc = mc + uLINK(i).c * uLINK(i).m;
        m_G = m_G + uLINK(i).m;
    else
        m_l = m_l + uLINK(i).m;
    end
end
p_G = mc./m_G;
r_G = p_G' - p_zmp;

plot3(p_G(1), p_G(2), p_G(3), 'go');
plot3(ones(1,501)*p_G(1), ones(1,501)*p_G(2), [0:500], 'g.');
plot3(p_zmp(1), p_zmp(2), p_zmp(3), 'ro');
plot3(ones(1,501)*p_zmp(1), ones(1,501)*p_zmp(2), [0:500], 'r-');
%}

% % modify the results correctly
%{
for p = 1:6
    % load files
    savefile = ['C:\Graduation_Project\One_Step_V8_improving\vision2.0_man\record_feasible_thrusts\record_feasible_thrust', int2str(p),'.mat'];
    load(savefile);

    % update the robot's pose 
    q_swing_op = the_swing_optimized;
    q_swing_op(6) = 0;
    q_swing_op(end) = 0;
    write(q_swing_op);
    
    % calculate the main body's CoM
    p_zmp = expected_zmp;
    mc = 0; m_G =0; m_l = 0;
    for i = 1:15
        if i ~= 6 && i ~= 7 && i~= 8 % eliminating the left foot's weight
            mc = mc + uLINK(i).c * uLINK(i).m;
            m_G = m_G + uLINK(i).m;
        else
            m_l = m_l + uLINK(i).m;
        end
    end
    p_G = mc./m_G;
    r_G = p_G' - p_zmp;
    
    % store the real results
    dist_r_G_optimized = abs(r_G(2));
    save(savefile, 'dist_r_G_optimized', '-append');
    
end
%}

% derive the formulas for calculating the ducted fan's thrust under different conditions 
%{
g = 9.8;
D = 100;
m_G =0; m_l = 0;
for i = 1:15
    if i ~= 6 && i ~= 7 && i~= 8
        m_G = m_G + uLINK(i).m;
    else
        m_l = m_l + uLINK(i).m;
    end
end

p = 1;

    % load the storage file for different configurations
    savefile = ['record_feasible_thrusts/record_feasible_thrust', int2str(p), '.mat'];
    load(savefile);

    % draw the function f
    figure;
    for R_G = -580:580
        ankle_theta = atan(...
                      ( mu_k * avg_r * g * (m_G * traj_radius + m_G * D + m_l * D - m_G * R_G) + angular_acc * (traj_radius + D) * (m_G * R_G ^2 + m_l * traj_radius ^2) ) ...
                      /((m_l * traj_radius + m_G * R_G) * (traj_radius + D) * g) ...
                      );
        f = (m_G * R_G + m_l * traj_radius) * g / ((traj_radius + D) * cos(ankle_theta));   
        plot(R_G, f, 'b.');
        hold on;
    end
%     axis([0, 120, 0, 30]);
    grid on;
    hold on;
    
    % display the experimental results
    [sorted_dist_r_G_stg, ascend_index] = sort(dist_r_G_stg);
    for kkk = 1:size(sorted_dist_r_G_stg)
        if f_stg(ascend_index(kkk)) < 15
            plot(-sorted_dist_r_G_stg(kkk), f_stg(ascend_index(kkk)), 'g.');
        else
            plot(sorted_dist_r_G_stg(kkk), f_stg(ascend_index(kkk)), 'g.');            
        end
    end
     if f_optimized < 15
            plot(-dist_r_G_optimized, f_optimized, 'rx');
        else
            plot(dist_r_G_optimized, f_optimized, 'rx');            
        end
    title(['angular acc = ', int2str(angular_acc*180/pi), ...
        ', mu_k = ', num2str(mu_k), ', traj radius = ', num2str(traj_radius), ...
        ', traj height = ', num2str(traj_height), '.']);
    xlabel('Distance from the main body''s CoM to the rotational axis'); ylabel('Needed thrust/N');
    
%}

% temporary: record the dist_r_G_stg
%{
dist_r_G_stg = [];
for kk = 1:length(f_stg)
    write(the_stg(kk, :));
    
    p_zmp = expected_zmp;
    mc = 0; m_G =0; m_l = 0;
    for i = 1:15
        if i ~= 6 && i ~= 7 && i~= 8 % eliminating the left foot's weight
            mc = mc + uLINK(i).c * uLINK(i).m;
            m_G = m_G + uLINK(i).m;
        else
            m_l = m_l + uLINK(i).m;
        end
    end
    p_G = mc./m_G;
    r_G = p_G' - p_zmp;
    
    dist_r_G_stg = [dist_r_G_stg; abs(r_G(2))];
end
save(savefile, 'dist_r_G_stg', '-append');
%}

%}

%% Transition phase
% Make the right hip and ankle in alignment, preparing for the swing phase   
the_trans_ini = the_for(end, :);
the_trans_end = the_swing(1, :);

steps = 50;

% initialization
the_trans = ones(steps, 14);

% assign values of the angles respectively
for j = 1:length(the_trans_ini)
    if the_trans_ini(j) == the_trans_end(j)
        the_trans(:, j) = the_trans(:, j) * the_trans_end(j);
    else
        the_trans(:, j) = the_trans_ini(j): (the_trans_end(j) - the_trans_ini(j)) / (steps - 1) : the_trans_end(j);
    end
end

% record relevant parameters
ref_trans = [];
position_trans = [];
% position_com = [];
% position_main_body_com = [];
pos_left_ducted_fan_trans = [];
for i = 1:steps

    % update the robot's state
    write(the_trans(i, :));
    
    % record data
    position_trans = [position_trans; uLINK(1).p'];
    pos_left_ducted_fan_trans = [pos_left_ducted_fan_trans; uLINK(16).p'];
    ref_trans = [ref_trans; 2];
    
end


%% Ending phase
% Make the both legs in upstraight pose
% Under the hypothesis that the left foot has been set to be horizontal to the ground
the_ending_ini = the_swing(end, :);
the_ending_end = [90, 0, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, 0, -90];

steps = 50;

% initialization
the_ending = ones(steps, 14);

% assign values of the angles respectively
for j = 1:length(the_ending_ini)
    if the_ending_ini(j) == the_ending_end(j)
        the_ending(:, j) = the_ending(:, j) * the_ending_end(j);
    else
        the_ending(:, j) = the_ending_ini(j): (the_ending_end(j) - the_ending_ini(j)) / (steps - 1) : the_ending_end(j);
    end
end

% record relevant parameters
ref_ending = [];
position_ending = [];
pos_left_ducted_fan_ending = [];
for i = 1:steps

    % update the robot's state
    write(the_ending(i, :));
    
    % record data
    position_ending = [position_ending; uLINK(1).p'];
    pos_left_ducted_fan_ending = [pos_left_ducted_fan_ending; uLINK(16).p'];
    ref_ending = [ref_ending; 2];
    
end


%% Post-processing, take into account of the reference point(??????)
% combine the joint configurations of 3 processes into a whole
the = [the_for; the_trans; the_swing; the_ending];
position = [position_for; position_trans; position_swing; position_ending];
ref = [ref_for; ref_trans; ref_swing; ref_ending];
the_bound = the_b; 
position_bound = p_b; 
ref_bound = ref_b;


end

