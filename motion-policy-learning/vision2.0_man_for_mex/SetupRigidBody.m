function SetupRigidBody(j,feet_size,feet_com)
%% This function constructs the feet
% shape: dimension [x y z], org: origin [x0 y0 z0]

global uLINK

%% Render the feet, rectangle(8 vertices, each with a 3-D coodinate)
% Define coordinates w.r.t the local coordinate
vert = [0            0            0;
        0            feet_size(2) 0;
        feet_size(1) feet_size(2) 0;
        feet_size(1) 0            0;
        0            0            feet_size(3);
        0            feet_size(2) feet_size(3);
        feet_size(1) feet_size(2) feet_size(3);
        feet_size(1) 0            feet_size(3)]';% 3 X 8
% Transform to a coordinate centered on CoM, thus it is geometrically
% symmetric w.r.t the CoM
vert(1,:) = vert(1,:) - feet_size(1)/2 + feet_com(1); 
vert(2,:) = vert(2,:) - feet_size(2)/2 + feet_com(2);
vert(3,:) = vert(3,:) - feet_size(3)/2 + feet_com(3);


face = [
   1 2 3 4;
   2 6 7 3;
   4 3 7 8;
   1 5 8 4;
   1 2 6 5;
   5 6 7 8;
]'; % ???


%% Record data
uLINK(j).vertex = vert; % feet vertices for display
uLINK(j).face   = face; % ???
uLINK(j).c      = feet_com; % center of mass w.r.t the 
uLINK(j).I = [1/12*(feet_size(2)^2 + feet_size(3)^2) 0 0;...
              0 1/12*(feet_size(1)^2 + feet_size(3)^2)  0;...
              0 0 1/12*(feet_size(2)^2 + feet_size(3)^2)] * uLINK(j).m; % moment of inertia

end