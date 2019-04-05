function [q]=Optimize_traj(Y)
global Length
Length=Y;
x0=[0;0;0;0;0;0].*pi/180;
A=[];b=[];
Aeq=[1 1 -1 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 1 1 -1 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0];
beq=[0 0 0 0 0 0]';

% lb=[-50;
%     -150;
%     -0;
%     -48;
%     -150;
%     -50
%     ].*pi/180;
lb=[-50;
    -150;
    -48;
    -48;
    -150;
    -50
    ].*pi/180;

ub=[100;
    130;
    115;
    115;
    130;
    100].*pi/180;
[x,fval]=fmincon(@fun_traj,x0,A,b,Aeq,beq,lb,ub,@mycon_traj);

q=[x(1),x(2),x(3),x(4),x(5),x(6)]*180/pi;