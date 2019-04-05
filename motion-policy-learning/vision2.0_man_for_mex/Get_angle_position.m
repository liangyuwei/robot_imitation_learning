function [q,p,ref]=Get_angle_position(P0,P,H)
global uLINK
q=[];
p=[];
ref=[];

base_p=[0 200 505];

   y2=[P0(2);(P0(2)+P(2))/2;P(2)];
   z2=[P0(3);    H  ;   P(3)];
   para=polyfit(y2,z2,2);

for y0=P0(2):3:P(2)
    
     
   z0=para(1)*y0^2+para(2)*y0+para(3);
  
    q1=Ln_IK_6Dof_find(y0,z0);
     write(q1);
     p1=[0 100 505+35 ]-uLINK(7).p'+[0 0 0];
     q=[q;q1,y0];
     p=[p;p1];
     ref=[ref;0];%%%%%%%%%%%%%%输出是以哪只脚为参考   0为左  1为右
end
% for y0=P(2)-200:3:P(2)
%     z0=0;
%     q1=Ln_IK_6Dof_find(y0,z0);
%      write(q1);
%      p1=[0 100 505+35 ]-uLINK(7).p';
%      q=[q;q1];
%      p=[p;p1];
%      ref=[ref;0];%%%%%%%%%%%%%%输出是以哪只脚为参考   0为左  1为右
% end