%% Test the method of calculating rotation from two vectors... see if the rotations are unique or not (Look-At method)

% Construct a descired rotation(ground truth) to move one frame to another
disp('>>>> Initial setup');
a = unit([1, 1, 1]');
eul_imposed = [pi/4, -pi/3, pi/6]; % [pi/2, 0, 0]; %
R_imposed = eul2rotm(eul_imposed, 'XYZ');
b = R_imposed * a;
disp(['Point 1: ', num2str(a')]);
disp(['Point 2: ', num2str(b')]);
disp('Imposed rotation: ');
R_imposed
disp(['Imposed euler angles: ', num2str(eul_imposed)]);


% Obtain the z vectors for both frames, and use them to calcualte euler angles by cross product + theta method...(Look-at method..)    
disp('>>>> Reconstruction')
A = unit(a);
B = unit(b);
C = unit(cross(a, b)); % rotation axis
theta = acos(dot(A, B)); % rotation angle by the law of cosine
R = axang2rotm([C', theta]); % corresponding rotation matrix
eul_calcuated1 = rotm2eul(R, 'XYZ');
eul_calcuated2 = tr2rpy(R);
disp('Calcualted rotation: '); 
R
disp(['Calculated euler angles(1): ', num2str(eul_calcuated1)]);
disp(['Calculated euler angles(2): ', num2str(eul_calcuated2)]);
disp(['Rotated Point 1: ', num2str((R*a)')]);


% Plot the resultant frames..?
b_calcualted = R*a;
length = 0.5;
figure;

% subplot(1, 2, 1);
plot3(a(1), a(2), a(3), 'bo'); hold on; grid on;
plot3(b(1), b(2), b(3), 'bo');
quiver3(b(1), b(2), b(3), R_imposed(1, 1) * length, R_imposed(2, 1) * length, R_imposed(3, 1) * length, 'r', 'LineWidth', 1, 'MaxHeadSize', 0.5);
quiver3(b(1), b(2), b(3), R_imposed(1, 2) * length, R_imposed(2, 2) * length, R_imposed(3, 2) * length, 'g', 'LineWidth', 1, 'MaxHeadSize', 0.5);
quiver3(b(1), b(2), b(3), R_imposed(1, 3) * length, R_imposed(2, 3) * length, R_imposed(3, 3) * length, 'b', 'LineWidth', 1, 'MaxHeadSize', 0.5);
% title('Actually imposed rotation', 'FontSize', 16);
% xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16);

% subplot(1, 2, 2);
plot3(a(1), a(2), a(3), 'bo'); hold on; grid on;
plot3(b_calcualted(1), b_calcualted(2), b_calcualted(3), 'b*');
quiver3(b_calcualted(1), b_calcualted(2), b_calcualted(3), R(1, 1) * length, R(2, 1) * length, R(3, 1) * length, 'r--', 'LineWidth', 1, 'MaxHeadSize', 0.5);
quiver3(b_calcualted(1), b_calcualted(2), b_calcualted(3), R(1, 2) * length, R(2, 2) * length, R(3, 2) * length, 'g--', 'LineWidth', 1, 'MaxHeadSize', 0.5);
quiver3(b_calcualted(1), b_calcualted(2), b_calcualted(3), R(1, 3) * length, R(2, 3) * length, R(3, 3) * length, 'b--', 'LineWidth', 1, 'MaxHeadSize', 0.5);
plot3([a(1), b_calcualted(1)], [a(2), b_calcualted(2)], [a(3), b_calcualted(3)], 'b--');

axis([-0.8, 0.6, -0.4, 1.0, 0.0, 1.4]);
view(-40, 55);
title('Actually imposed rotation and Calculated rotation'); %, 'FontSize', 16);
xlabel('x', 'FontSize', 16); ylabel('y', 'FontSize', 16); zlabel('z', 'FontSize', 16);



