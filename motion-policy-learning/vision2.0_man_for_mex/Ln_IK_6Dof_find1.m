%%%%%%%%%%%%%%%%%%%%%¸ø¶¨the1 ºÍ the3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function the = Ln_IK_6Dof_find(Y,Z,L1,L2,d)
global det_th

the1=(((Y-d)/(6*(L1+L2))))^0.9*(pi/2);
the3=(((Y-d)/(2*(L1+L2))))^(2)*(pi/2);

the1=the1-60*pi/180;
the3=the3+40*pi/180;
if the3>=pi/2
    the3=pi/2;
end


y=L1*sin(the3)+L2*sin(the1);
z=L1*cos(the3)+L2*cos(the1);
the2=the3-the1;


r=[Y-y-d,Z-z]';

R=norm(r);
while((L1+L2-R)<=0)
   the1=the1+3.14/72;
   if the3<=pi/2
   the3=the3+3.14/72;
   end
   y=L1*sin(the3)+L2*sin(the1);
  z=L1*cos(the3)+L2*cos(the1);
  the2=the3-the1;


  r=[Y-y-d,Z-z]';

  R=norm(r);
end

alp=atan2(abs(r(1)),abs(r(2)));
if alp>=pi/2
    alp=pi-alp;
end

b1=L2^2+R^2-L1^2;
b2=2*R*L2;

beta=-abs(acos(b1/b2));

the6=(alp+beta);

g1=L2^2+L1^2-R^2;
g2=2*L1*L2;
gama=acos(g1/g2);
the5 =-(pi-gama);

the4=the6-the5;

%the =[the1 the2 the3 the4 the5 the6]'*180/pi;
the =[0 0 the3 -the2 -the1 0      0 0 the4 the5 -the6 0]*180/pi;
end