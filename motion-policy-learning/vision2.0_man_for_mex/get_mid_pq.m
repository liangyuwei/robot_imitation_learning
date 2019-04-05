function [th,po,ref]=get_mid_pq(q0,pos)
global uLINK
th=[];
po=[];
ref=[];

Y=pos(2);
Z=pos(3);


L1=220;
L2=140;
d=200;

the4=asin((Y-d)/(2*(L1+L2)))*180/pi;
the5=0;

step=50;


det_the5=[q0(10):(the5-q0(10))/step:the5];
det_the4=[q0(9):(the4-q0(9))/step:the4];

det_the6=-(det_the5+det_the4);
for i=1:step-1
 
y=Y-d+L1*sin(det_the6(i)*pi/180)-L2*sin(det_the4(i)*pi/180);
z=Z+L1*cos(det_the6(i)*pi/180)+L2*cos(det_the4(i)*pi/180);

r=[y,z]' ;
R=norm(r);


alp=atan2(r(1),r(2))*180/pi;


b1=L2^2+R^2-L1^2;
b2=2*R*L2;

beta=acos(b1/b2)*180/pi;

det_the3=(alp+beta);

g1=L2^2+L1^2-R^2;
g2=2*L1*L2;
gama=acos(g1/g2);
det_the2 =abs(gama)*180/pi-180;

det_the1=-(det_the2+det_the3);

the0=[0 0 det_the3 det_the2 det_the1 0   0 0 det_the4(i) det_the5(i) det_the6(i) 0];% 
 write(the0);
 p1=[0 100 505+35 ]-uLINK(7).p';
 po=[po;p1];
 th=[th;the0];
 ref=[ref;0];%%%%%%%%%%%%%%输出是以哪只脚为参考   0为左  1为右
end
m=step;
for i=abs(m)-1:-1:1
    the00=[0 0 th(i,9) th(i,10) th(i,11) 0   0 0 th(i,3) th(i,4) th(i,5) 0];
    write(the00);
     p1=[0 100 505+35 ]-uLINK(13).p'+[0 Y Z];
      po=[po;p1];
     th=[th;the00];
     ref=[ref;1];%%%%%%%%%%%%%%输出是以哪只脚为参考   0为左  1为右
end
