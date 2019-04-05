function [the,pos1,ref]=similar_traj(p,ther,Pend)
global uLINK pr
the=[];
pos1=[];
ref=[];
th=[];
p(1,:)=[0 200 0];
p(end,:)=Pend;
max(p(:,3))
num=find(p(:,3)==max(p(:,3)))

% x=[200  p(num,2) p(ceil(1.1*num),2) p(ceil(1.2*num),2) p(ceil(1.3*num),2) p(ceil(1.4*num),2) p(ceil(1.5*num),2) p(ceil(1.6*num),2) Pend(2)];
% y=[0    p(num,3) p(ceil(1.1*num),3) p(ceil(1.2*num),3) p(ceil(1.3*num),3) p(ceil(1.4*num),3) p(ceil(1.5*num),3) p(ceil(1.6*num),3) Pend(3)];
% x=[200  p(num-40:end-5,2)' Pend(2)-3 Pend(2)-2 Pend(2)-1 Pend(2)];
% y=[35   p(num-40:end-5,3)' Pend(3)+38 Pend(3)+37 Pend(3)+36 Pend(3)+35];
x=[200  p(num-40:end,2)'];
y=[35   p(num-40:end,3)'];

coe=polyfit(x,y,3);

m=length(p(:,1))-1;
k=1;
for y=200:(x(end)-200)/m:x(end)
   %   z=coe(1)*y^5+coe(2)*y^4+coe(3)*y^3+coe(4)*y^2+coe(5)*y^1+coe(6);
   % z=coe(1)*y^3+coe(2)*y^2+coe(3)*y^1+coe(4);
 z=coe(1)*y^3+coe(2)*y^2+coe(3)*y+coe(4);
    q=similar_IK_traj(ther(k,:),y,z);
    k=k+1;
    th=[th;q,y];
    
end

for i=1:length(th(:,1))
  the0=[90 0 th(i,3) th(i,4) th(i,5) 0   -90 0 -th(i,9) th(i,10) th(i,11) 0];
  
  write(the0);
    uLINK(7).p'
     p1=[0 100 505+35 ]-uLINK(7).p'+[0 -32 -187];
     pos1=[pos1;p1];
     the=[the;the0,th(i,13)];
     ref=[ref;0];
end