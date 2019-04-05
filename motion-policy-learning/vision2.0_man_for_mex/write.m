function write(q)
global th
%a=find(q);
ToRad = pi/180;

for i=1:length(q)
    th(i)=q(i) * ToRad;
end
wholebody; % change joint configuration