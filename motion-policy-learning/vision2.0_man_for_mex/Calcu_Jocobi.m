function J=Calcu_Jocobi(i,k)
global uLINK;
jsize = k-i+1;
target =uLINK(k).p;
J=zeros(6,jsize);
for n=1:jsize
    j=i+n-1;
    a=uLINK(j).R*uLINK(j).a;
    J(:,n)=[cross(a,target-uLINK(j).p);a];
end
    
