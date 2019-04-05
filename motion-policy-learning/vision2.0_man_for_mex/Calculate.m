function [the,pos,ref]=Calculate(the_ini,the_mid1,the_mid2,the_end,pos_ini,pos_end,Height)
%%%%%the:Joints angle%%%%%%%%%
%%%%%pos: the position of base as refere to reference%%%%%%%%%
%%%%%ref: the reference%%%%%%%%%
global uLINK;

the=[];
pos=[];
ref=[];

Lfoot_ref=pos_ini-[0 200 0];%=[0 0 0]
Rfoot_ref=pos_end;




[the0,pos0,ref0]=Inital_End(the_ini,the_mid1,Lfoot_ref);

[the1,pos1,ref1]=Get_angle_position(pos_ini,pos_end,Height);

[the2,pos2,ref2]=get_mid_pq(the1(end,:),pos_end);

[the3,pos3,ref3]=Get_inv_angle_position(pos_end,the1);

[the4,pos4,ref4]=Inital_End(the_mid2,the_end,pos_end);

the=[the0;the1;the2;the3;the4];
%pos=[pos0;pos1;pos2;pos3;pos4];
ref=[ref0;ref1;ref2;ref3;ref4];

for i=1:12
    the(:,i)=smooth(the(:,i),21);   
end
the(:,4)=-(the(:,3)+the(:,5));
the(:,10)=-(the(:,9)+the(:,11));

for j=1:length(the(:,1))
    write(the(j,:));
    if ref(j)==0
    p1=[0 100 505+35 ]-uLINK(7).p'+Lfoot_ref;
      pos=[pos;p1];
    elseif ref(j)==1
     p1=[0 100 505+35 ]-uLINK(13).p'+pos_end;
      pos=[pos;p1];
    end
end