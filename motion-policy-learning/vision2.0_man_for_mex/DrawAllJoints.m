function DrawAllJoints(j)
global uLINK
radius    = 25;
len       = 60;
joint_col =0;

if j ~= 0 
    
    if ~isempty(uLINK(j).vertex)
        vert = uLINK(j).R * uLINK(j).vertex; % rigid body transformation...??????
        for k = 1:3
            vert(k,:) = vert(k,:) + uLINK(j).p(k); % adding x,y,z to all vertex
        end
        DrawPolygon(vert, uLINK(j).face,joint_col);
    end
    
    hold on;
    
    i = uLINK(j).mother;
    if i ~= 0 && i ~= 14 && i ~= 7
        Connect3D(uLINK(i).p,uLINK(j).p,'k',6);
    end
    
    if j ~= 8 && j ~= 15
        DrawCylinder(uLINK(j).p, uLINK(j).R * uLINK(j).a, radius,len, joint_col);
    end
    
    DrawAllJoints(uLINK(j).child);
    DrawAllJoints(uLINK(j).sister);
   

    grid on;
end
