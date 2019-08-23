q = zeros(1, 6); %[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
q = -[0, pi/4, -pi/2, 0, -pi/2, 0];
T_base = [eul2rotm([0, 0, -pi/4]), [-0.06, 0.235, 0.395]'; 0, 0, 0, 1 ];
% L0 = Link('revolute', 'a', 0, 'alpha', 0, 'd', 0, 'offset', 0);
% L1 = Link('revolute', 'a', 0.235, 'alpha', 0, 'd', 0.395, 'offset', pi/2);
% L2 = Link('revolute', 'a', 0.06, 'alpha', pi/4, 'd', 0, 'offset', 0);
L3 = Link('revolute', 'a', 0, 'alpha', 0, 'd', 0.089159, 'offset', 0);
L4 = Link('revolute', 'a', 0, 'alpha', pi/4, 'd', 0.13585, 'offset', -0.75*pi);
L5 = Link('revolute', 'a', -0.425, 'alpha', 0, 'd', -0.1197, 'offset', -pi/2);
L6 = Link('revolute', 'a', 0.39225, 'alpha', 0, 'd', 0.093, 'offset', 0);
L7 = Link('revolute', 'a', 0, 'alpha', -pi/2, 'd', 0.09465, 'offset', -pi/2);
L8 = Link('revolute', 'a', 0, 'alpha', -pi/2, 'd', 0.0823, 'offset', 0);
% L9 = Link('revolute', 'a', 0, 'alpha', pi/2, 'd', 0, 'offset', 0);
l_arm = SerialLink([L3, L4, L5, L6, L7, L8], 'base', T_base);
l_arm.name = 'left_arm';
l_arm.display();
l_arm.plot(q, 'tilesize', 2);

T = l_arm.fkine(q);
eul = rotm2eul(T.R)
pos = T.t'


