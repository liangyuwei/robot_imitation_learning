function [the,posi,ref]=Opt_traj(yend)
global uLINK F
th=[];
the=[];
posi=[];
ref=[];

for y=200:2:yend
   q=Optimize_traj(y)
   th=[th;q,y];
end

for i=1:length(th(:,1))
  the0=[90 0 th(i,3) -th(i,2) -th(i,1) 0   -90 0 th(i,4) th(i,5) -th(i,6) 0];
  
  write(the0);
     p1=[0 100 505+35 ]-uLINK(7).p'+[0 0 0];
     posi=[posi;p1];
     the=[the;the0,th(i,7)];
     ref=[ref;0];


end