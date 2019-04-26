function w = rot2omega(R)
%% This function transforms rotation matrix to rotation vector.

if R == eye(3)
    w = [0, 0, 0]';
    
else
    theta = acos((R(1, 1) + R(2, 2) + R(3, 3) - 1) / 2);
    w = [R(3, 2) - R(2, 3), R(1, 3) - R(3, 1), R(2, 1) - R(1, 2)]' * theta / (2 * sin(theta));
end

end