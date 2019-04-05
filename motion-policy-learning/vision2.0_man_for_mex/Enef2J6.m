function p=Enef2J6(rot,pit,yaw,x,y,z)  %Endefector Position Transformation
if nargin<7 Lef=0; end %The default of Lef is 0

%%%%Endefector Position Transformation
 AhR=rpy2rot(rot,pit,yaw);
 Ahp=[x,y,z]';
 Ah=[AhR,Ahp;0,0,0,1];
 Rz=[1,0,0,0;
     0,1,0,0;
     0,0,1,0;
     0,0,0,1];
 A7=Ah*Rz;
 
 p=A7(1:3,4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

