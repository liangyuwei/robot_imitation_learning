%% Initialization
% Clear screen
close all
clear
clearvars -GLOBAL % remove all global variables
clc;

% Set up parameters
global th uLINK Pend pos pl pr info1 det H_1 Torque_all Force ref_phase expected_zmp
global com_traj left_end_traj right_end_traj base_traj com_main_traj

ToRad = pi/180;

Pend = [0, 920, 0];
th = zeros(14,1); % joint angles
pos = [0, 100, 500]'; % the position of the base w.r.t the world frame
pl = [];
pr = [];
ref_phase = 0; % initialize

expected_zmp = [0, 940, 0]; % the CENTER of the right foot!!!


info1 = [];
det = -190; % distance from the ducted-fan to the heel
H_1 = 0;

Force = [];
Torque_all = [];

% parameters for the swing phase
global traj_height traj_radius angular_acc mu_k
traj_height = 300; %40 1+ 220 * cos(the_trans_end(12)*ToRad) + 140 * cos(the_trans_end(10)) - 40;
traj_radius = 380; % the extreme position(580) % 200 + (140 + 220) + 20; 
angular_acc = 1 * ToRad; % in radius per second(!!!)
mu_k = 0.3; % kinematic friction coefficient

% Build up robot's model
wholebody;

%% Trajectory planning
% Generate joint trajectory(there are more...)
[the, position, ref, the_bound, position_bound, ref_bound, f] = Opt_H([0 920 0],0);
% ref indicates the process: 0 - right leg; 1 - left leg; 2 - transition

pl = [];
pr = [];

% f is the collection of thrusts needed to stay balance during the swing phase 
% f

%% Execute(Display) joint motion, step by step!!!
% initialize com_traj for storage
com_traj = [];
left_end_traj = [];
right_end_traj = [];
base_traj = [];
com_main_traj = [];

% execute planned motion
for i=1:length(the(:,1)) % 'the' stores the joint configurations of the whole process
                         % 'position' stores the location of the base w.r.t ???
    if i~=length(the(:,1))
        SetPose(the(i,:), position(i,:)', ref(i), 1);
    else
        SetPose(the(i,:), position(i,:)', ref(i), 0);
    end
end

%% display the trajectory of the com
figure;
hold on;
% draw the trajectories of the base and the left ducted fan independently
plot3(com_traj(1:end, 1), com_traj(1:end, 2), com_traj(1:end, 3), 'b.');
plot3(left_end_traj(1:end, 1), left_end_traj(1:end, 2), left_end_traj(1:end, 3), 'r.');
plot3(right_end_traj(1:end, 1), right_end_traj(1:end, 2), right_end_traj(1:end, 3), 'r.');
plot3(base_traj(1:end, 1), base_traj(1:end, 2), base_traj(1:end, 3), 'k.');
plot3(com_main_traj(1:end, 1), com_main_traj(1:end, 2), com_main_traj(1:end, 3), 'g.');
plot3(uLINK(15).p(1), uLINK(15).p(2), uLINK(15).p(3), 'yx'); % pivot point
% connect corresponding points to form lines
for k = 146:size(com_traj,1)
    plot3([com_traj(k, 1), left_end_traj(k, 1)], [com_traj(k, 2), left_end_traj(k, 2)], [com_traj(k, 3), left_end_traj(k, 3)], 'g-');
    plot3([com_traj(k, 1), right_end_traj(k, 1)], [com_traj(k, 2), right_end_traj(k, 2)], [com_traj(k, 3), right_end_traj(k, 3)], 'g-');
    
end
% display relevent information
title('Trajectory of the CoM during swing phase');
xlabel('x'); ylabel('y'); zlabel('z');
grid on;


%% Plot trajectory
figure(2);
plot3(ones(length(pl(:,1)),1),pl(:,2),(pl(:,3)-40),'r.',ones(length(pr(:,1)),1),pr(:,2),(pr(:,3)-40),'b.');
axis([-600,500,-200,1099,-200,400]);
grid on;
ylabel('Length/mm');
zlabel('Height/mm');
text(0,100,-50,' starting position','Fontsize',15);
text(0,800,50,' goal position','Fontsize',15);
set(gca,'Fontsize',15)

%% Display statistical results
% Plot the change of joints' angles
figure(3)
m = 1:length(the(:,1));
s = 1:50:length(the(:,1)); % for sparse plot
plot(m, the(:,3), 'r', m, the(:,4), 'm', m, the(:,5), 'g', m, the(:,9), 'c', ...
    m, the(:,10), 'b', m, the(:,11), 'k');
hold on;
plot(s, the(s,3), '*r', s, the(s,4), '^m',  s, the(s,5), 'sg', s, the(s,9), '+c', ...
    s, the(s,10), 'ob', s, the(s,11), 'hk');
xlabel('The number of point on trajectory');
ylabel('The change of Joints angle/radius');
set(gca,'Fontsize',15);
legend('Lhip2','Lhip3','Lknee','Rhip2','Rhip3','Rknee'); % ???
grid on;

% Plot thrust of ducted fans
figure(4)
m = 1:length(Force(:,1));
plot(m, Force(:,1), m, Force(:,2));
xlabel('The number of points on trajectory');
ylabel('Thrust of ducted fans/N');
set(gca,'Fontsize',15);
grid on;

% Display torques of joints, only 6 joints are moved in this algorithm(check out the lateral view)
figure(5)
m=1:length(Torque_all(:,1));
plot(m, Torque_all(:,1), m, Torque_all(:,2), m, Torque_all(:,3), ...
     m, Torque_all(:,4), m, Torque_all(:,5), m, Torque_all(:,6));
xlabel('The number of points on trajectory');
ylabel('Torque of every joint/Nm');
set(gca,'Fontsize',15)
grid on;

%% Record results
fp=fopen('angle_force.txt','a');
for i=1:length(the(:,1))
    fprintf(fp,'%5.1f ,%5.3f ,%5.3f ,%5.3f ,%5.3f ,%5.3f ,%5.3f ,%5.3f ,%5.3f\r\n',...
        [i,the(i,3:5).*pi/180,the(i,9:11).*pi/180],Force(i,:));
end
fclose(fp);
