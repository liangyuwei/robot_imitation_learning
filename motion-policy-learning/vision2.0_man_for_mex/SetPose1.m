function SetPose1(q)
global th;
global uLINK;
global pos pl pr F info1
a=find(q);

for i=1:length(a)-1
    
    th(a(i))=q(a(i))*pi/180;
end
    write(q);
    p1=[0 100 505+35 ]-uLINK(7).p'+[0 0 0];
    pos=p1';
    wholebody;
    p_left=uLINK(7).p';
   
    pl=[pl;p_left];
    p_right=uLINK(13).p';
   
    pr=[pr;p_right];
    
    
    if p_right(2)>q(14)
    s=length(pr(:,1));
    M=0.3;
    mc=pos.*uLINK(1).m;
    F=0;
   
for i=2:13
    
    mc=mc+uLINK(i).c.*uLINK(i).m;
    M=M+uLINK(i).m;
end
    
    det_pos=p_right-p_left;
    com=mc./M;
%     plot3( com(1),com(2),com(3),'ro');hold on;
   % if p_left(2)<1||p_left(3)<41
    f=M*9.8*(com(2)+100)/(abs(det_pos(2))+200);
%     elseif p_right(2)<801||p_right(3)<41
%     f=M*9.8*(800-com(2)+100)/(abs(det_pos(2))+200)   
   % end
   if f<q(15)&&p_right(3)<(q(13)+5)&&p_right(3)>(q(13)-5)
    F=[F;f];
    h=(220*cos(q(1))+140*cos(q(3))-220*cos(q(6))-140*cos(q(4)));
    
    info1=[info1;q,p_right(3),p_right(2),f];
   end
    end  
%     h = plot3([p_right(1) p_right(1)],[p_right(2)+100 p_right(2)+100],[p_right(3)-35 p_right(3)+f*3-40],'b');
%          set(h,'LineWidth',4)
%          text(p_right(1),p_right(2),p_right(3)+60, ['F=',num2str(f,'%5.3f'),'N']);
%     
    
%     if p_left(3)>41||p_right(3)>41
%         if p_right(2)>200&&p_right(3)>41
%          h = plot3([p_right(1) p_right(1)],[p_right(2)+100 p_right(2)+100],[p_right(3)-35 p_right(3)+f*3-40],'b');
%          set(h,'LineWidth',4)
%          text(p_right(1),p_right(2),p_right(3)+60, ['F=',num2str(f,'%5.3f'),'N']);
%         elseif p_left(2)>200&&p_left(3)>41
%          h = plot3([p_left(1) p_left(1)],[p_left(2)-100 p_left(2)-100],[p_left(3)-35 p_left(3)+f*3-40],'b');
%          set(h,'LineWidth',4)
%          text(p_left(1),p_left(2),p_left(3)+60, ['F=',num2str(f,'%5.3f'),'N']);
%         end
%     end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%   shape=[1200 400 20];
%   com=[0 00 -15];
%   com1=[0 800 -15];
%  vert = [
%    0      0      0;
%    0      shape(2) 0;
%    shape(1) shape(2) 0;
%    shape(1) 0      0;
%    0      0      shape(3);
%    0      shape(2) shape(3);
%    shape(1) shape(2) shape(3);
%    shape(1) 0      shape(3);
% ]';
% 
% 
% vert(1,:) = vert(1,:) -shape(1)/2 + com(1);
% vert(2,:) = vert(2,:) -shape(2)/2 + com(2);
% vert(3,:) = vert(3,:) -shape(3)/2 + com(3);
% 
% vert1 = [
%    0      0      0;
%    0      shape(2) 0;
%    shape(1) shape(2) 0;
%    shape(1) 0      0;
%    0      0      shape(3);
%    0      shape(2) shape(3);
%    shape(1) shape(2) shape(3);
%    shape(1) 0      shape(3);
% ]';
% 
% 
% vert1(1,:) = vert1(1,:) -shape(1)/2 + com1(1);
% vert1(2,:) = vert1(2,:) -shape(2)/2 + com1(2);
% vert1(3,:) = vert1(3,:) -shape(3)/2 + com1(3);
% 
% 
% face = [
%    1 2 3 4;
%    2 6 7 3;
%    4 3 7 8;
%    1 5 8 4;
%    1 2 6 5;
%    5 6 7 8;
% ]';
% DrawPolygon(vert, face,0);
% DrawPolygon(vert1, face,0);
%     
%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%     
% axis([-600,500,-300,1100,-300,600]);
%  view(90,0);
% xlabel('X');%set(gca,'XAxisLocation','top');
% ylabel('Y'); %set(gca,'YDir','reverse');
% zlabel('Z'); %set(gca,'ZDir','reverse');
% wholebody;
% DrawAllJoints(2);
% 
% plot3(pl(:,1),pl(:,2),(pl(:,3)-35),'r*',pr(:,1),pr(:,2),(pr(:,3)-35),'g*');
% hold on;
% drawnow;


