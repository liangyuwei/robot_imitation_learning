function the=Get_mid_angle(q,Pini,Pend)

Pend=Pend+[0 200 0]
% L1=220;
% L2=140;
% the=q;
% y=Pend(2)-220*sin(q(11))-140*sin(q(9))-200-Pini(2);
% z=Pend(3)+200*cos(q(11))-140*cos(q(9))-Pini(3);
% r=[y z];
% R=norm(r);
% alp=atan2(y,z);
% b1=L2^2+R^2-L1^2;
% b2=2*L2*R;
% beta=acos(b1/b2);
% g1=L1^2+L2^2-R^2;
% g2=2*L1*L2;
% gama=pi-acos(g1/g2);
% 
% the(3)=alp+beta;
% the(4)=gama;
% the(5)=-(the(3)+the(4));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


L1=220;
L2=140;


the6=q(11);
the4=q(9);


y=Pend(2)-220*sin(the6)-140*sin(the4)-200-Pini(2)
z=Pend(3)+220*cos(the6)+140*cos(the4)-Pini(3)
the5=-(the4+the6);

r=[y,z]'

R=norm(r);

% if (L1+L2-R)<0
% while(1)
%    the6=the6+.314/72;
%    if the4>pi*115/180
%    the4=the4+.314/72;
%    end
%   
% y=Pend(2)-220*sin(the6)-140*sin(the4)-200-Pini(2)
% z=Pend(3)+200*cos(the6)+140*cos(the4)-Pini(3)
% the5=-(the4+the6);
% 
% r=[y,z]';
% 
% R=norm(r);
% if (L1+L2-R)>0
%     break;
% end
% 
% end
% end
    

alp=atan2(r(1),(r(2)))


b1=L2^2+R^2-L1^2;
b2=2*R*L2;

beta=acos(b1/b2);

the3=(alp+beta)

g1=L2^2+L1^2-R^2;
g2=2*L1*L2;
gama=acos(g1/g2);
the2 =pi-abs(gama);

the1=-(the3+the2);


the =[pi/2 0 the3 -the2 the1 0    -pi/2 0 the4 the5 the6 0]*180/pi;  
end