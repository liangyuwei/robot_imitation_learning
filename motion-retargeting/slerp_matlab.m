function [ q3 ] = slerp_matlab( q1, q2, t )
%SLERP quaternion slerp
%   computes the slerp of value t between quaternions q1 and q2
%   q1 and q2 need to be column vectors
%   Copied from https://gist.github.com/simonlynen/5349167

q1 = q1 ./ norm(q1);
q2 = q2 ./ norm(q2);

one = 1.0 - eps;
d = q1'*q2;
absD = abs(d);

if(absD >= one)
    scale0 = 1 - t;
    scale1 = t;
else
    % theta is the angle between the 2 quaternions
    theta = acos(absD);
    sinTheta = sin(theta);
    
    scale0 = sin( ( 1.0 - t ) * theta) / sinTheta;
    scale1 = sin( ( t * theta) ) / sinTheta;
end
if(d < 0)
    scale1 = -scale1;
end

q3 = scale0 * q1 + scale1 * q2;
q3 = q3 ./ norm(q3);

end

function [] = testslerp()
q1 = [1, 1, 0, 0]';
q2 = [1, 0, 0, 0]';
q3_gt = [0.92387953251128673848, 0.38268343236508978178, 0, 0]';
t = 0.5;

q3 = slerp_matlab(q1, q2, t);

diff = q3 - q3_gt
end
