function [the,position,ref]=Inital_End(q0,q1,ref_pos)
global uLINK
the=[];
position=[];
ref=[];
step=50;


%th_lh=q0(1):(q1(1)-q0(1))/step:q1(1);

th1=q0(3):(q1(3)-q0(3))/step:q1(3);
th2=q0(4):(q1(4)-q0(4))/step:q1(4);
th3=q0(5):(q1(5)-q0(5))/step:q1(5);

%th_rh=q0(7):(q1(7)-q0(7))/step:q1(7);
th4=q0(9):(q1(9)-q0(9))/step:q1(9);
th5=q0(10):(q1(10)-q0(10))/step:q1(10);
th6=q0(11):(q1(11)-q0(11))/step:q1(11);

if ref_pos==[0 0 0]
    num=7;
else num=13;
end

for i=1:step+1
    the0=[0 0 th1(i) th2(i) th3(i) 0   0 0 th4(i) th5(i) th6(i) 0];
    write(the0);
    p1=[0 100 505+35 ]-uLINK(num).p'+ref_pos;
    position=[position;p1];
    
    the=[the;the0];
    
    ref=[ref;floor(num/13)];%%%%%%%%%%%%%%输出是以哪只脚为参考   0为左  1为右


end