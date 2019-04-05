function [the]=similar_IK_traj(q,Y,Z)
L1=220;
L2=140;
d=200;

the1=-q(5)*pi/180;
the3=-q(3)*pi/180;
the2=-(the1+the3);

y=L1*sin(the1)-L2*sin(the3);
z=L1*cos(the1)+L2*cos(the3);

r=[Y-y-d,Z-z-40]';

R=norm(r);



if (L1+L2-R)<0
while(1)
   the1=the1+.314/72;
   if the3>-pi/2
   the3=the3-.314/72;
   end
  
  y=L1*sin(the1)-L2*sin(the3);
  z=L1*cos(the1)+L2*cos(the3);
  the2=-(the3+the1);


  r=[Y-y-d,Z-z-40]';

  R=norm(r);
if (L1+L2-R)>0
    break;
end

end
end


alp=atan2(r(1),abs(r(2)));


b1=L1^2+R^2-L2^2;
b2=2*R*L1;

beta=acos(b1/b2);

the6=-(alp+beta);

g1=L2^2+L1^2-R^2;
g2=2*L1*L2;
gama=acos(g1/g2);
the5 =pi-abs(gama);

the4=-(the6+the5);

the =[pi/2 0 -the3 -the2 -the1 0      -pi/2 0 -the4 the5 the6 0]*180/pi  % the4 -the5 -the6
end

