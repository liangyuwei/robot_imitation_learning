%% This file constructs the robot's model, and is used to modify its joint configuration

%% Initialize related parameters
ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
global uLINK;
global th;
global joint_feet_displacement


%% Construct the robot's model
% q - joint variables; m - link mass; sister, child, mother --- related concepts,
% tree-form structure; c - CoM; b - ; a - joint axis vector; b - relative
% position vector.
uLINK = struct('name','body'   , 'm', 0.3, 'sister', 0, 'child', 2, 'b', [0 0 0]', 'a', UZ, 'q', 0, 'c', [0 0 0]');

uLINK(2) = struct('name', 'Lhip1',  'm', 0.298, 'sister', 9, 'child', 3, 'b', [0 -100 0]', 'a', UZ, 'q', th(1), 'c', [0 100 0]');
uLINK(3) = struct('name', 'Lhip2', 'm', 0.4, 'sister', 0, 'child', 4, 'b', [0  0 -100]',  'a', UX, 'q', th(2), 'c', [0 0 50]');
uLINK(4) = struct('name', 'Lhip3', 'm', 0.245, 'sister', 0, 'child', 5, 'b', [0 0 0]', 'a', UY,'q', th(3), 'c', [0 0 0]');
uLINK(5) = struct('name', 'Lknee', 'm', 0.667, 'sister', 0, 'child', 6, 'b', [0 0 -140]' , 'a', UY, 'q', th(4), 'c', [0 0 70]');
uLINK(6) = struct('name', 'Lankle1', 'm', 0.718, 'sister', 0, 'child', 7, 'b',[0 0 -220]' ,'a', UY, 'q', th(5), 'c', [0 0 110]');
uLINK(7) = struct('name', 'Lankle2', 'm', 0.632, 'sister', 0, 'child', 8, 'b',[0 0 0]' ,'a', UX, 'q', th(6), 'c', [0 60 35]');
% uLINK(8) should be included in the pose calculation, but 'q' should be set 0 for now.
uLINK(8) = struct('name', 'LfootCenter', 'm', 0, 'sister', 0, 'child', 0, 'b',[-20 0 -joint_feet_displacement]' ,'a', UZ, 'q', th(7), 'c', [0 0 0]');

uLINK(9) = struct('name','Rhip1', 'm', 0.298, 'sister', 0, 'child', 10, 'b',[0  100 0]'   ,'a',UZ,'q',th(8),'c',[0 100 0]');
uLINK(10) = struct('name','Rhip2', 'm', 0.4, 'sister', 0, 'child',11, 'b',[0  0   -100]',  'a',UX,'q',th(9),'c',[0 0 50]');
uLINK(11) = struct('name','Rhip3' , 'm', 0.245, 'sister', 0, 'child',12, 'b',[0  0   0]'   ,'a',UY,'q',th(10),'c',[0 0 0]');
uLINK(12) = struct('name','Rknee' , 'm', 0.667, 'sister', 0, 'child',13, 'b',[0  0  -140]' ,'a',UY,'q',th(11),'c',[0 0 70]');
uLINK(13) = struct('name','Rankle1', 'm', 0.718, 'sister', 0, 'child',14, 'b',[0  0  -220]' ,'a',UY,'q',th(12),'c',[0 0 110]');
uLINK(14) = struct('name','Rankle2', 'm', 0.632, 'sister', 0, 'child', 15, 'b',[0  0   0]' ,'a',UX,'q',th(13),'c',[0 60 35]');
uLINK(15) = struct('name','RfootCenter', 'm', 0, 'sister', 0, 'child', 0, 'b',[-20  0   -joint_feet_displacement]' ,'a',UZ,'q',th(14),'c',[0 0 0]');


% the left and right ducted fan's position 'b' is w.r.t the ankle coordinate, 
% i.e. uLINK(7) and uLINK(14) respectively 
uLINK(16) = struct('name','Lfan', 'm', 0, 'sister', 17, 'child', 0, 'b', [-100  0   -40]' ,'a',UZ,'q',0,'c',[0 0 0]');
uLINK(17) = struct('name','Rfan', 'm', 0, 'sister', 16, 'child', 0, 'b', [-100  0   -40]' ,'a',UZ,'q',0,'c',[0 0 0]');


% Set up relationships between adjacent joints
if ~isfield(uLINK, 'mother')
    % run it only for the first time
    FindMother(1);
end

% Assign IDs to variables named by uLINK(i).name, e.g. Lankle1 = 6;
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

% Build the robot's feet
feet_size = [220 80 18]; % size of the feet(length, width, height)
feet_com = [-20  0 9]';%-32]'; % center of mass of the feet w.r.t ???
SetupRigidBody(LfootCenter, feet_size, feet_com);
SetupRigidBody(RfootCenter, feet_size, feet_com);

% Forward kinematics modeling
% calculate the absolute transforms of all frames
global ref_phase swing_pivot
if ref_phase == 0
    % left foot based motion
    left_foot_center = [-120, 0, 0]';
    swing_pivot = left_foot_center;
    
    % update the foot's pose and position
    uLINK(LfootCenter).p = left_foot_center; % set the position and pose of the right foot
    uLINK(LfootCenter).R = [cos(uLINK(8).q), -sin(uLINK(8).q), 0;
                            sin(uLINK(8).q),  cos(uLINK(8).q), 0;
                                          0,                0, 1];
    ForwardKinematics_left_base;
    
else
    % right foot based motion    
    right_foot_center = [-120, 940, 0]';
    swing_pivot = right_foot_center;
    
    % update the foot's pose and position
    % it seems that it doesn't matter what uLINK(RfootCenter).p/R would be....
    % it serves only for the purpose of recording... 
    % However, its angle of rotation does matter, i.e. uLINK(15).q, which is used to evaluate the ankle's pose w.r.t the world frame 
    uLINK(RfootCenter).p = right_foot_center; % set the position and pose of the right foot
    uLINK(RfootCenter).R = [cos(uLINK(15).q - pi/2), -sin(uLINK(15).q - pi/2), 0;
                            sin(uLINK(15).q - pi/2),  cos(uLINK(15).q - pi/2), 0;
                                           0,                 0,               1];
                     
    ForwardKinematics_right_base; % still use the right ankle as the base!!!
    
end

%% Special-purpose : for the positioning of the ducted-fans
% same expression for both situations
uLINK(16).p = uLINK(7).R * uLINK(16).b + uLINK(7).p;
uLINK(16).R = uLINK(7).R * Rodrigues(uLINK(16).a, uLINK(16).q);
uLINK(17).p = uLINK(14).R * uLINK(17).b + uLINK(14).p;
uLINK(17).R = uLINK(14).R * Rodrigues(uLINK(17).a, uLINK(17).q);
