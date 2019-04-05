% Original

function q = IK_leg(Body,D,A,B,Foot)

r = Foot.R' * (Body.p + Body.R * [0 D 0]'- Foot.p);  % ‘«Žñ‚©‚çŒ©‚½ŒÒŠÖß

C = norm(r); 
c5 = (C^2-A^2-B^2)/(2.0*A*B);
if c5 >= 1 
    q5 = 0.0;
elseif c5 <= -1
    q5 = pi;
else
    q5 = acos(c5);  % knee pitch
end
q6a = asin((A/C)*sin(pi-q5));   % ankle pitch sub   

q7 = atan2(r(2),r(3));  % ankle roll -pi/2 < q(6) < pi/2
if q7 > pi/2, q7=q7-pi; elseif q7 < -pi/2, q7=q7+pi; end

if  r(3)~=0
q6 = -atan2(r(1),sign(r(3))*sqrt(r(2)^2+r(3)^2)) -q6a; % ankle pitch
else
q6 = -atan2(r(1),1*sqrt(r(2)^2+r(3)^2)) -q6a;   
end



R = Body.R' * Foot.R * Rroll(-q7) * Rpitch(-q6-q5); %% hipZ*hipX*hipY
% q2  = atan2(R(1,3),R(3,3));   % hip pitch
% q3 = -asin(R(2,3));  % hip roll
% q4 = atan2( R(2,1), R(2,2));               % hip yaw
q2  = atan2(-R(1,2),R(2,2));   % hip pitch
cz=cos(q2);sz=sin(q2);
q3 = atan2(R(3,2),-R(1,2)*sz+R(2,2)*cz);  % hip roll
q4 = atan2( -R(3,1), R(3,3));               % hip yaw

q = [q2 q3 q4 q5 q6 q7]';



% function q = IK_leg(Body,D,A,B,Foot)
% % FR11=Foot.R(1,1);FR12 =Foot.R(1,2);FR13=Foot.R(1,3);FR21=Foot.R(2,1);FR22 =Foot.R(2,2);FR23=Foot.R(2,3);FR31=Foot.R(3,1);FR32=Foot.R(3,2);FR33=Foot.R(3,3);
% % FP1=Foot.p(1); FP2=Foot.p(2); FP3=Foot.p(3);
% % BR11=Body.R(1,1); BR12=Body.R(1,2);BR13=Body.R(1,3); BR21=Body.R(2,1); BR22=Body.R(2,2); BR23=Body.R(2,3); BR31=Body.R(3,1); BR32=Body.R(3,2); BR33=Body.R(3,3);
% % BP1=Foot.p(1); BP2=Foot.p(2); BP3= Foot.p(3);
% % 
% % FR=[FR11 FR12 FR13;FR21 FR22 FR23 ;FR31 FR32 FR33];
% % FP=[FP1 FP2 FP3]';
% % BR=[BR11 BR12 BR13; BR21 BR22 BR23; BR31 BR32 BR33];
% % BP=[BP1 BP2 BP3]';
% % 
% %  rz=BP+BR*[0 D 0]'-FP;
% %  r = FR'*rz
% %  f1=conj(BP1) - conj(FP1) + BR12*conj(D);
% %  r=...
% %  [(conj(FR11)*( f1) + conj(FR21)*(conj(BP2) - conj(FP2) + BR22*conj(D)) + conj(FR31)*(conj(BP3) - conj(FP3) + BR32*conj(D)));
% %  conj(FR12)*( f1) + conj(FR22)*(conj(BP2) - conj(FP2) + BR22*conj(D)) + conj(FR32)*(conj(BP3) - conj(FP3) + BR32*conj(D));
% %  conj(FR13)*( f1) + conj(FR23)*(conj(BP2) - conj(FP2) + BR22*conj(D)) + conj(FR33)*(conj(BP3) - conj(FP3) + BR32*conj(D))];
% 
% r0 =...
%  [conj(Foot.R(1,1))*(conj(Body.p(1)) - conj(Foot.p(1)) + Body.R(1,2)*conj(D)) + conj(Foot.R(2,1))*(conj(Body.p(2)) - conj(Foot.p(2)) + Body.R(2,2)*conj(D)) + conj(Foot.R(3,1))*(conj(Body.p(3)) - conj(Foot.p(3)) + Body.R(3,2)*conj(D))
%  conj(Foot.R(1,2))*(conj(Body.p(1)) - conj(Foot.p(1)) + Body.R(1,2)*conj(D)) + conj(Foot.R(2,2))*(conj(Body.p(2)) - conj(Foot.p(2)) + Body.R(2,2)*conj(D)) + conj(Foot.R(3,2))*(conj(Body.p(3)) - conj(Foot.p(3)) + Body.R(3,2)*conj(D))
%  conj(Foot.R(1,3))*(conj(Body.p(1)) - conj(Foot.p(1)) + Body.R(1,2)*conj(D)) + conj(Foot.R(2,3))*(conj(Body.p(2)) - conj(Foot.p(2)) + Body.R(2,2)*conj(D)) + conj(Foot.R(3,3))*(conj(Body.p(3)) - conj(Foot.p(3)) + Body.R(3,2)*conj(D))]'
%  
% 
%  r = Foot.R' * (Body.p + Body.R * [0 D 0]'- Foot.p)  
% C = norm(r);
% c5 = (C^2-A^2-B^2)/(2.0*A*B);
% if c5 >= 1 
%     q5 = 0.0;
% elseif c5 <= -1
%     q5 = pi;
% else
%     q5 = acos(c5);  % knee pitch
% end
% q6a = asin((A/C)*sin(pi-q5));   % ankle pitch sub
% 
% q7 = atan2(r(2),r(3));  % ankle roll -pi/2 < q(6) < pi/2
% if q7 > pi/2, q7=q7-pi; elseif q7 < -pi/2, q7=q7+pi; end
% q6 = -atan2(r(1),sign(r(3))*sqrt(r(2)^2+r(3)^2)) -q6a; % ankle pitch
% 
% 
% 
% R0=...
% [cos(q5+q6)*(Foot.R(1,1)*(Body.R(1,1)) + Foot.R(2,1)*(Body.R(2,1)) + Foot.R(3,1)*(Body.R(3,1))) + sin(q5+q6)*(cos(q7)*(Foot.R(1,3)*(Body.R(1,1)) + Foot.R(2,3)*(Body.R(2,1)) + Foot.R(3,3)*(Body.R(3,1))) + sin(q7)*( Foot.R(1,2)*(Body.R(1,1)) + Foot.R(2,2)*(Body.R(2,1)) + Foot.R(3,2)*(Foot.R(3,1)))),      cos(q7)*(Foot.R(1,2)*(Body.R(1,1)) + Foot.R(2,2)*(Body.R(2,1)) + Foot.R(3,2)*(Body.R(3,1))) - sin(q7)*(Foot.R(1,3)*(Body.R(1,1)) + Foot.R(2,3)*(Body.R(2,1)) + Foot.R(3,3)*(Body.R(3,1))),      cos(q5+q6)*(cos(q7)*(Foot.R(1,3)*(Body.R(1,1)) + Foot.R(2,3)*(Body.R(2,1)) + Foot.R(3,3)*conj(Body.R(3,1))) + sin(q7)*(Foot.R(1,2)*(Body.R(1,1)) + Foot.R(2,2)*(Body.R(2,1)) + Foot.R(3,2)*(Body.R(3,1)))) - sin(q5+q6)*(Foot.R(1,1)*(Body.R(1,1)) + Foot.R(2,1)*(Body.R(2,1)) + Foot.R(3,1)*(Body.R(3,1)));
%  cos(q5+q6)*(Foot.R(1,1)*(Body.R(1,2)) + Foot.R(2,1)*(Body.R(2,2)) + Foot.R(3,1)*(Body.R(3,2))) + sin(q5+q6)*(cos(q7)*(Foot.R(1,3)*(Body.R(1,2)) + Foot.R(2,3)*(Body.R(2,2)) + Foot.R(3,3)*(Body.R(3,2))) + sin(q7)*( Foot.R(1,2)*(Body.R(1,2)) + Foot.R(2,2)*(Body.R(2,2)) + Foot.R(3,2)*(Foot.R(3,2)))),      cos(q7)*(Foot.R(1,2)*(Body.R(1,2)) + Foot.R(2,2)*(Body.R(2,2)) + Foot.R(3,2)*(Body.R(3,2))) - sin(q7)*(Foot.R(1,3)*(Body.R(1,2)) + Foot.R(2,3)*(Body.R(2,2)) + Foot.R(3,3)*(Body.R(3,2))),      cos(q5+q6)*(cos(q7)*(Foot.R(1,3)*(Body.R(1,2)) + Foot.R(2,3)*(Body.R(2,2)) + Foot.R(3,3)*conj(Body.R(3,2))) + sin(q7)*(Foot.R(1,2)*(Body.R(1,2)) + Foot.R(2,2)*(Body.R(2,2)) + Foot.R(3,2)*(Body.R(3,2)))) - sin(q5+q6)*(Foot.R(1,1)*(Body.R(1,2)) + Foot.R(2,2)*(Body.R(2,2)) + Foot.R(3,1)*(Body.R(3,2)));
%  cos(q5+q6)*(Foot.R(1,1)*(Body.R(1,3)) + Foot.R(2,1)*(Body.R(2,3)) + Foot.R(3,1)*(Body.R(3,3))) + sin(q5+q6)*(cos(q7)*(Foot.R(1,3)*(Body.R(1,3)) + Foot.R(2,3)*(Body.R(2,3)) + Foot.R(3,3)*(Body.R(3,3))) + sin(q7)*( Foot.R(1,2)*(Body.R(1,3)) + Foot.R(2,2)*(Body.R(2,3)) + Foot.R(3,2)*(Foot.R(3,3)))),      cos(q7)*(Foot.R(1,2)*(Body.R(1,3)) + Foot.R(2,2)*(Body.R(2,3)) + Foot.R(3,2)*(Body.R(3,3))) - sin(q7)*(Foot.R(1,3)*(Body.R(1,3)) + Foot.R(2,3)*(Body.R(2,3)) + Foot.R(3,3)*(Body.R(3,3))),      cos(q5+q6)*(cos(q7)*(Foot.R(1,3)*(Body.R(1,3)) + Foot.R(2,3)*(Body.R(2,3)) + Foot.R(3,3)*conj(Body.R(3,3))) + sin(q7)*(Foot.R(1,2)*(Body.R(1,3)) + Foot.R(2,2)*(Body.R(2,3)) + Foot.R(3,2)*(Body.R(3,3)))) - sin(q5+q6)*(Foot.R(1,1)*(Body.R(1,3)) + Foot.R(2,3)*(Body.R(2,3)) + Foot.R(3,1)*(Body.R(3,3)))]
%    R = Body.R' * Foot.R * Rroll(-q7) * Rpitch(-q6-q5) %% hipZ*hipX*hipY
% 
% 
% 
% 
% 
% q2  = atan2(-R(1,2),R(2,2)) ;  % hip yaw
% cz = cos(q2); sz = sin(q2);
% q3 = atan2(R(3,2),-R(1,2)*sz + R(2,2)*cz) ; % hip roll
% 
% % q4=asin(R(1,3)*cz+R(2,3)*sz)
%   q4 = atan2( -R(3,1), R(3,3));               % hip pitch
% 
% q = [q2 q3 q4 q5 q6 q7]';














