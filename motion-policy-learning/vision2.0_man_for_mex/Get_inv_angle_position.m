function [the,pos,ref]=Get_inv_angle_position(position,th)
global uLINK
pos=[];
the=[];
ref=[];
m=length(th(:,1));
for i=m:-1:1
  the0=[0 0 th(i,9) th(i,10) th(i,11) 0   0 0 th(i,3) th(i,4) th(i,5) 0];
 write(the0);
     p1=[0 100 505+35 ]-uLINK(13).p'+[0 position(2) position(3)];
     pos=[pos;p1];
     the=[the;the0];
     ref=[ref;1];
end