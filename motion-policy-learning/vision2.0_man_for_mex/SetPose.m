function SetPose(q,posi,ref,fcla)
%% This function updates the robot's state and displays the motion.

%% Load global variables and set up related parameters
global th;
global uLINK;
global pl pr det Pend Torque_all  Force
global ref_phase 
global com_traj left_end_traj right_end_traj base_traj com_main_traj
global expected_zmp mu_k avg_r angular_acc

% apply new joint angles and change them to radius
for i = 1:length(q)
    th(i) = q(i) * pi/180;
end


% indicate the phase, for left-leg phase the uLINK(body).R should be modified  
ref_phase = ref;

%% Update the robot's state
wholebody;

%% Set the critical points
% obtain the absolute positions of all the coordinate frames
p_left=uLINK(7).p'; % Left ankle2
pl=[pl;p_left]; % pl is empty initially
p_lankle=uLINK(6).p';
p_lknee=uLINK(5).p';
p_lhip=uLINK(4).p';
p_right=uLINK(14).p'; % Right ankle2
pr=[pr;p_right]; % pr is empty initially
p_rankle=uLINK(13).p';
p_rknee=uLINK(12).p';
p_rhip=uLINK(9).p';

%% Calculation of CoM
% CoM of the main body / whole body
mc = 0; mc1 = 0; m_G =0; m_l = 0;
for i = 1:15
    if i ~= 6 && i ~= 7 && i~= 8 % eliminating the left foot's weight
        mc = mc + uLINK(i).c * uLINK(i).m;
        m_G = m_G + uLINK(i).m;
    else
        mc1 = mc1 + uLINK(i).c * uLINK(i).m;
        m_l = m_l + uLINK(i).m;
    end
end
p_G = mc ./ m_G; % CoM of the main body
com = (mc + mc1) ./ (m_l + m_G); % CoM of the whole body
M = m_l + m_G; % Mass of the whole body

% plot the CoM
plot3(com(1), com(2), com(3), 'ro');
hold on;


%% record the critical points' coordinates for later display in Go_man.m
com_traj = [com_traj; com'];
left_end = uLINK(7).p';
left_end_traj = [left_end_traj; left_end];
right_end = uLINK(17).p';
right_end_traj = [right_end_traj; right_end];
base_traj = [base_traj; uLINK(1).p'];
com_main = p_G';
com_main_traj = [com_main_traj; com_main];

%% Calculate the required thrusts from the ducted fans
% 'ref' = {0, 1, 2}, indicating the three processes    
% Note that : f - RIGHT, f2 - LEFT
if ref==0
    %  the [right-leg] process
    
    % calculate torque and the arm of force
    W = M * 9.8 * com(2); % gravitational moment
    L1 = (abs(uLINK(7).p(2) - uLINK(14).p(2)) + 2 * 100 + det); % distance between the left heel and the right ducted fan
    L2 = abs(det); % distance between the right ducted fan and the right heel
    
    % distribute needed moments for the two ducted fans to provide
    W1 = W; % moment for the right ducted fan's thrust to balance
    W2 = 0; % moment for the left ducted fan's thrust to balance, the same direction as the gravitational moment since the pivot point is the left heel
    
    % RIGHT ducted fan's thrust
    f = W1 / L1;
   
    % LEFT ducted fan's thrust
    f2 = 0; % W2 / L2;
    
elseif ref == 1
    % the [left-leg] process = [swing] + [transition] phase
    
    % Required thrust from the [RIGHT] ducted fan
    f = 0;
    
    % relevant parameters
    D = 100;
  
    % position of the left ankle w.r.t ZMP
    p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
    p_zmp = expected_zmp;
    r_L = p_L - p_zmp;
    r_Ly = r_L(2);

    % CoM of the whole robot without the part of the swing leg's foot
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
    r_Gy = r_G(2);

    g = 9.8; % gravitational acceleration

    % Required thrust from the left ducted fan
    ankle_theta = atan(...
                  ( mu_k * avg_r * g * (m_G*r_Ly-m_G*D-m_l*D-m_G*r_Gy) - angular_acc * (-r_Ly + D) * (m_G * r_Gy ^2 + m_l * r_Ly ^2) ) ...
                 /((m_l*r_Ly+m_G*r_Gy) * (- r_Ly + D) * g) ...
                      );
                     
    % error!!! f2 is related to 'angular_acc', which is not updated in this function!!!                
    f2 = (m_G * r_Gy + m_l * r_Ly) * g / ((r_Ly - D) * cos(ankle_theta));

else
    % ref_phase == 2, for the fall-down phases that treat the support foot as the base  
    
    %% for the right ducted fan
    f = 0;

    %% for the left ducted fan
    % relevant parameters
    D = 100;
  
    % position of the left ankle w.r.t ZMP
    p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
    p_zmp = expected_zmp;
    r_L = p_L - p_zmp;
    R_L = (r_L(1)^2+r_L(2)^2)^0.5;

    % CoM of the whole robot without the part of the swing leg's foot
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
    R_G = (r_G(1)^2+r_G(2)^2)^0.5;

    g = 9.8; % gravitational acceleration
    
    % By the torque equation: f2 * (R_L + D) - m_l * g * R_L - m_G * g * R_G = 0  
    f2 = (m_G * R_G + m_l * R_L) * g / (R_L + D);
    
end

%% Calculate the the joint torque during 3 processes
if ref==0
    % the [right-leg] process
    f_point = p_right + [0 100 -40]; % right ankle + dist_ankle_fan --> the position of the right ducted fan w.r.t the base
    
    T6=Torque([14],p_rankle,f,f_point);
    T5=Torque([14 13],p_rknee,f,f_point);
    T4=Torque([14 13 12],p_rhip,f,f_point);
    
    T3=Torque([14 13 12 11 10 9 1 2 3],p_lhip,f,f_point);
    T2=Torque([14 13 12 11 10 9 1 2 3 4],p_lknee,f,f_point);
    T1=Torque([14 13 12 11 10 9 1 2 3 4 5],p_lankle,f,f_point);

elseif ref == 1
    % the [left-leg] process
    f2_point=p_left-[0 100 0];
    
    T1=Torque([7],p_left,f2,f2_point);
    T2=Torque([7 6],p_lknee,f2,f2_point);
    T3=Torque([7 6 5],p_lhip,f2,f2_point);
    
    T4=Torque([7 6 5 4 3 2 1 9 10 ],p_rhip,f2,f2_point);
    T5=Torque([7 6 5 4 3 2 1 9 10 11],p_rknee,f2,f2_point);
    T6=Torque([7 6 5 4 3 2 1 9 10 11 12],p_rankle,f2,f2_point);
    
else %ref==2
% the [transition] process
    % Set all to zero, actually they are not 0, but it's not our concern 
    T1=0;
    T2=0;
    T3=0;
    T4=0;
    T5=0;
    T6=0;
    
end


%% Draw the thrusts provided by ducted fans, and ZMP
if ref==0
    % the [right-leg] process
    % draw thrusts
    h = plot3([p_right(1) p_right(1)],[p_right(2)+100 p_right(2)+100],[p_right(3)-35 p_right(3)+f*3-40],'g');
    set(h,'LineWidth',4)
    text(p_right(1),p_right(2)+30,p_right(3)+40, ['F1=',num2str(f,'%5.2f'),'N']);
    h1 = plot3([p_left(1) p_left(1)],[p_left(2)-100 p_left(2)-100],[p_left(3)-25 p_left(3)+f2*3-25],'r');
    set(h1,'LineWidth',4)
    text(p_left(1),p_left(2)-180,p_left(3)+40, ['F2=-',num2str(f2,'%5.2f'),'N']);
    f2 = -f2;
    % ZMP
    plot3(p_left(1)+100,p_left(2)-100-det,p_left(3)-40,'bo');
    
elseif ref==1
    % the [left-leg] process
    % draw thrusts
    h = plot3([p_right(1) p_right(1)],[p_right(2)+100 p_right(2)+100],[p_right(3)-35 p_right(3)+f*3-40],'r');
    set(h,'LineWidth',4)
    text(p_right(1),p_right(2)+160,p_right(3)+60, ['F1=-',num2str(f,'%5.2f'),'N']);
    f=-f;
    h1 = plot3([p_left(1) p_left(1)],[p_left(2)-100 p_left(2)-100],[p_left(3)-25 p_left(3)+f2*3-25],'g');
    set(h1,'LineWidth',4)
    text(p_left(1),p_left(2)-180,p_left(3)+40, ['F2=',num2str(f2,'%5.2f'),'N']);
    % ZMP(blue dot)
    plot3(p_right(1)+100,p_right(2)+100+det,p_right(3)-40,'bo');
    
else % ref==2
    % the [transition] process
    % do nothing...
end

% Record the results
Torque_all=[Torque_all;T1,T2,T3,T4,T5,T6];
Force = [Force;f,f2];

%% Draw feet
shape=[1200 400 20];
com=[0 00 -10];
com1=[Pend(1) Pend(2) Pend(3)]+[0 0 -10];
vert = [
    0      0      0;
    0      shape(2) 0;
    shape(1) shape(2) 0;
    shape(1) 0      0;
    0      0      shape(3);
    0      shape(2) shape(3);
    shape(1) shape(2) shape(3);
    shape(1) 0      shape(3);
    ]';


vert(1,:) = vert(1,:) -shape(1)/2 + com(1);
vert(2,:) = vert(2,:) -shape(2)/2 + com(2);
vert(3,:) = vert(3,:) -shape(3)/2 + com(3);

vert1 = [
    0      0      0;
    0      shape(2) 0;
    shape(1) shape(2) 0;
    shape(1) 0      0;
    0      0      shape(3);
    0      shape(2) shape(3);
    shape(1) shape(2) shape(3);
    shape(1) 0      shape(3);
    ]';


vert1(1,:) = vert1(1,:) -shape(1)/2 + com1(1);
vert1(2,:) = vert1(2,:) -shape(2)/2 + com1(2);
vert1(3,:) = vert1(3,:) -shape(3)/2 + com1(3);


face = [
    1 2 3 4;
    2 6 7 3;
    4 3 7 8;
    1 5 8 4;
    1 2 6 5;
    5 6 7 8;
    ]';
DrawPolygon(vert, face,0);
DrawPolygon(vert1, face,0);

%% Draw the robot
set (gcf,'Position',[0,0,800,1600], 'color','w')
% set (gcf,'Position',[9 121 1268 585], 'color','w')

% axis([-600,500,-400,1600,-300,750]);
axis([-700,700,-200,1200,-300,750]);

view(90,0);
xlabel('X');
ylabel('Length/mm');
zlabel('Height/mm'); 
set(gca,'Fontsize',15)

% update joint configuration and robot's pose
% wholebody; % already done in the fore-part of this function

% draw the robot!!!
DrawAllJoints(2);

% the left and right ankles(invisible...)
plot3(pl(:,1),pl(:,2),(pl(:,3)-35),'r.',pr(:,1),pr(:,2),(pr(:,3)-40),'b.');
hold on;

% update figure...
drawnow;

% clear screen
if(fcla)
    cla;
end

end