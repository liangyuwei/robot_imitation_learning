function h_x = elementary_trajectory_compute(ele_traj_struc, x)
%% This function computes elementary trajectory h(x) with the given x.
% Note that this function doesn't modify the function shape, only computes function value.
% elementary trajectory h(x) is modeled as fifth-order polynomial, minimum-jerk trajectory.    
% Input: ele_traj_struc - stores the segment point (lower floor of a segment's x interval), and stores coefficients of fifth-order polynomial.       


%% Check input data validity
if x <0 || x >1
    disp('Computing elementary trajectory h(x): Input error, must be a number between 0 and 1!!!');
    exit(-1);
end

%% Check if any via-points exist for now 
if size(ele_traj_struct.floor, 2) == 2
    % no via-points exist now, elementary trajectory is a line connecting the start point to the goal     
    start = ele_traj_struc.start;
    goal = ele_traj_struc.goal;
    h_x = interp1([0, 1], [start, goal], x);
else
    % via-points exist, use fifth-order polynomial to calculate the function value at x    
    % Find which interval x belongs to ( x belongs to [x0, x1) )
    id = find(x < ele_traj_struc.floor, 1);
%     x0 = ele_traj_struc.floor(id-1);
%     x1 = ele_traj_struc.floor(id);
    h_x = polyval(flip(ele_traj_struc.coeff(id-1)), x); % polyval, coefficients in descending order
end

end