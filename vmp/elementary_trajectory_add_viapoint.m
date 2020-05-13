function ele_traj_struct = elementary_trajectory_add_viapoint(ele_traj_struct, x_via, y_via, y_seq, f_seq, x_seq, quat_or_not)
%% This function adds via-points to elementary trajectory, and fits fifth-order polynomials to the two newly added segments.
% x_seq - canonical time
% y_seq - NO!!!! we need the current y_seq here!!! derived by the current h(x) and f(x) before the modification ;;;;;;original imitation data, used for interpolating to obtain accurate values of y(x0) and y(x1)
% f_seq - shape modulation data, used for interpolation


%% Find which interval x belongs to
id = find(x_via < ele_traj_struct.floor, 1); % id of ceiling
x0 = ele_traj_struct.floor(id-1);
x1 = ele_traj_struct.floor(id); % x0 <= x_via < x1

if quat_or_not
    % quaternion data (simply add via-point quaternion to model)
    [~, I] = find(x_via < x_seq, 1);
    f_via = quatinterp(f_seq(:, I-1)', f_seq(:, I)', x_via, 'slerp')';
    h_via = quatmultiply(y_via', quatinv(f_via'));
    % store
    ele_traj_struct.quat = [ele_traj_struct.quat; h_via];
    ele_traj_struct.floor = [ele_traj_struct.floor, x_via];
    % sort the order
    [~, I] = sort(ele_traj_struct.floor);
    ele_traj_struct.floor = ele_traj_struct.floor(I);
    ele_traj_struct.quat = ele_traj_struct.quat(I, :);
else
    %% Prepare data for later solving equation for polynomial's coefficients
    % original imitation trajectory
    dt = x_seq(2) - x_seq(1);
    dy_seq = gradient(y_seq) / dt;
    ddy_seq = gradient(dy_seq) / dt;
    y_x0 = interp1(x_seq, y_seq, x0);
    y_x1 = interp1(x_seq, y_seq, x1);
    y_x_via = y_via; %interp1(x_seq, y_seq, x_via); % use y_via
    dy_x0 = interp1(x_seq, dy_seq, x0);
    dy_x1 = interp1(x_seq, dy_seq, x1);
    dy_x_via = interp1(x_seq, dy_seq, x_via);
    ddy_x0 = interp1(x_seq, ddy_seq, x0);
    ddy_x1 = interp1(x_seq, ddy_seq, x1);
    ddy_x_via = interp1(x_seq, ddy_seq, x_via);
    
    % shape modulation
    df_seq = gradient(f_seq) / dt;
    ddf_seq = gradient(df_seq) / dt;
    f_x0 = interp1(x_seq, f_seq, x0);
    f_x1 = interp1(x_seq, f_seq, x1);
    f_x_via = interp1(x_seq, f_seq, x_via);
    df_x0 = interp1(x_seq, df_seq, x0);
    df_x1 = interp1(x_seq, df_seq, x1);
    df_x_via = interp1(x_seq, df_seq, x_via);
    ddf_x0 = interp1(x_seq, ddf_seq, x0);
    ddf_x1 = interp1(x_seq, ddf_seq, x1);
    ddf_x_via = interp1(x_seq, ddf_seq, x_via);
    
    % coefficient matrix
    A_x = @(x) [1, x, x^2, x^3, x^4, x^5;
        0, 1, 2*x, 3*x^2, 4*x^3, 5*x^4;
        0, 0, 2, 6*x, 12*x^2, 20*x^3];
    
    
    %% First segment
    A1 = [A_x(x0); A_x(x_via)]; % h(), dh(), ddh(), h(), dh(), ddh()
    b1 = [y_x0 - f_x0;
        dy_x0 - df_x0;
        ddy_x0 - ddf_x0;
        y_x_via - f_x_via;
        dy_x_via - df_x_via;
        ddy_x_via - ddf_x_via];
    alpha0 = (A1' * A1) \ A1' * b1; % column vector
    
    
    %% Second segment
    A2 = [A_x(x_via); A_x(x1)]; % h(), dh(), ddh(), h(), dh(), ddh()
    b2 = [y_x_via - f_x_via;
        dy_x_via - df_x_via;
        ddy_x_via - ddf_x_via;
        y_x1 - f_x1;
        dy_x1 - df_x1;
        ddy_x1 - ddf_x1];
    alpha1 = (A2' * A2) \ A2' * b2;
    
    
    %% Store the results
    % add 1 id to .floor, replace one line and add one line to .coeff
    ele_traj_struct.floor = [ele_traj_struct.floor, x_via];
    ele_traj_struct.coeff(id-1, :) = alpha0'; % replace the one closer to x0
    ele_traj_struct.coeff = [ele_traj_struct.coeff; alpha1']; % add the one closer to x1
    % sort the order
    [~, I] = sort(ele_traj_struct.floor);
    ele_traj_struct.floor = ele_traj_struct.floor(I);
    ele_traj_struct.coeff = ele_traj_struct.coeff(I, :);
    
end

end