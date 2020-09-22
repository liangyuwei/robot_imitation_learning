function err = mean_rot_error(R1, R2)
%% This function calcualtes the average orientation error.

num_datapoints = size(R1, 3);
assert(num_datapoints == size(R2, 3), 'Number of rotation matrices should be consistent!!!');

err = [];

for i = 1 : num_datapoints
    err = [err, abs(acos((trace(R1(:, :, i)' * R2(:, :, i))-1)/2))];
end

err = mean(err);


end