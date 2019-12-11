%% Build robot model
global uLINK th
th = zeros(6, 1);
UX = [1 0 0]'; UY = [0 1 0]'; UZ = [0 0 1]';
uLINK = struct('name','body', 'mom', 0, 'child', 2, 'b', [0 0 0]', 'a', UZ, 'q', 0);
uLINK(2) = struct('name', 'shoulder_link', 'mom', 1, 'child', 3, 'b', [0 0 0.089159]', 'a', UZ, 'q', th(1));
uLINK(3) = struct('name', 'upper_arm_link', 'mom', 2, 'child', 4, 'b', [0  0.13585 0]',  'a', UY, 'q', th(2));
uLINK(4) = struct('name', 'forearm_link', 'mom', 3, 'child', 5, 'b', [0.425 -0.1197 0]', 'a', UY,'q', th(3));
uLINK(5) = struct('name', 'wrist_1_link', 'mom', 4, 'child', 6, 'b', [0.39225 0 0]' , 'a', UY, 'q', th(4));
uLINK(6) = struct('name', 'wrist_2_link', 'mom', 5, 'child', 7, 'b', [0 0.093 0]' ,'a', -UZ, 'q', th(5));
uLINK(7) = struct('name', 'wrist_3_link', 'mom', 6, 'child', 8, 'b', [0 0 -0.09465]' ,'a', UY, 'q', th(6)); % child = 0 indicates that the calculation of forward kinematics should stop here
uLINK(8) = struct('name', 'ee_link', 'mom', 7, 'child', 0, 'b', [0 0.0823 0]', 'a', UY, 'q', pi/2); % eef, if necessary

%% FK for left arm
q_left = zeros(6, 1);
th = q_left;
uLINK(1).R = eul2rotm([0, 0, -pi/4]);%eye(3);    
uLINK(1).p = [-0.06, 0.235, 0.395]';
update_robot_joint(true);
ForwardKinematics(2);
disp(['Wrist position: ', num2str(uLINK(8).p')]); 
disp(['Wrist quaternion(w,x,y,z): ', num2str(rotm2quat(uLINK(8).R))]); % quaternion here is (w,x,y,z)   
disp(['Corresponding euler angles are: ' num2str(rotm2eul(uLINK(8).R))]);
disp(['Elbow position: ', num2str(uLINK(4).p')]);
disp(['Elbow quaternion(w,x,y,z): ', num2str(rotm2quat(uLINK(4).R))]); % quaternion here is (w,x,y,z)   

tmp = [0.7071         0    -0.7071
      -0.5000   -0.7071    -0.5000
      -0.5000    0.7071    -0.5000];



%% FK for right arm

 