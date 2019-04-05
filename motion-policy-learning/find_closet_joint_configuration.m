function best_q0_index = find_closet_joint_configuration(x_shoulder, xe_goal, xw_goal, xe_dataset, xw_dataset)
%% This function searches over the whole dataset and picks the closet joint configuration.


%% Preparation
% define distance function
% d = @(x, y) sqrt(sum((x - y) .^ 2));
num_samples = size(xe_dataset, 1); % is it?

% interpolate points of the line crossing x_shoulder, xe_goal and xw_goal
N_s_e = 50; N_e_w = 50; % interpolation points
X_goal_interp = zeros(3, N_s_e + N_e_w);
for xi = 1 : 3 % one and only one
    % interpolate points between x_shoulder and xe_goal
    if (x_shoulder(xi) - xe_goal(xi))^2 < eps
        X_goal_interp(xi, 1 : N_s_e) = x_shoulder(xi) * ones(1, N_s_e);
    else
        X_goal_interp(xi, 1 : N_s_e) = x_shoulder(xi) : (xe_goal(xi) - x_shoulder(xi)) / (N_s_e - 1) : xe_goal(xi);
    end
    % interpolate points between xe_goal and xw_goal
    if (xw_goal(xi) - xe_goal(xi))^2 < eps
        X_goal_interp(xi, (N_s_e + 1) : end) = xe_goal(xi) * ones(1, N_e_w);
    else
        X_goal_interp(xi, (N_s_e + 1) : end) = xe_goal(xi) : (xw_goal(xi) - xe_goal(xi)) / (N_e_w - 1) : xw_goal(xi);
    end
end
% figure; plot3(X_goal_interp(1, :), X_goal_interp(2, :), X_goal_interp(3, :), 'r.');
% grid on;


%% Enumerate over the dataset
X_demo_interp = zeros(3, N_s_e + N_e_w);
for id = 1 : num_samples
%     for t = 1: T ......% .... a sample consists of joint trajectory points over time t = 1, ..., T !!!!
    % do linear interpolation
    xe_demo = xe_dataset(id, :);
    xw_demo = xw_dataset(id, :);
%     figure; plot3( [x_shoulder(1), xe_goal(1), xw_goal(1)], ...
%                    [x_shoulder(2), xe_goal(2), xw_goal(3)], ...
%                    [x_shoulder(3), xe_goal(2), xw_goal(3)], 'b-'); grid on;
    for xi = 1 : 3
        % interpolate points between x_shoulder and xe_goal
        if (x_shoulder(xi) - xe_demo(xi))^2 < eps
            X_demo_interp(xi, 1 : N_s_e) = x_shoulder(xi) * ones(1, N_s_e);
        else
            X_demo_interp(xi, 1 : N_s_e) = x_shoulder(xi) : (xe_demo(xi) - x_shoulder(xi)) / (N_s_e - 1) : xe_demo(xi);
        end
        % interpolate points between xe_goal and xw_goal
        if (xw_demo(xi) - xe_demo(xi))^2 < eps
            X_demo_interp(xi, (N_s_e + 1) : end) = xe_demo(xi) * ones(1, N_e_w);
        else
            X_demo_interp(xi, (N_s_e + 1) : end) = xe_demo(xi) : (xw_demo(xi) - xe_demo(xi)) / (N_e_w - 1) : xw_demo(xi);
        end
    end
%     figure; plot3(X_demo_interp(1, :), X_demo_interp(2, :), X_demo_interp(3, :), 'r.'); 
%     grid on;

    % compute the Frechet distance between a sample and the goal
    [tmp_Frechet_dist, ~] = DiscreteFrechetDist(X_demo_interp', X_goal_interp'); % use L2 norm as default
    if t == 1 && id == 1
        min_Frechet_dist = tmp_Frechet_dist;
        best_q0_index_t = 1;
        best_q0_index_sample = 1;
    end
    if tmp_Frechet_dist < min_Frechet_dist
        % should return the sample index and the corresponding time t!!!
        min_Frechet_dist = tmp_Frechet_dist;
        best_q0_index_t = t;
        best_q0_index_sample = id;
    end
     
end