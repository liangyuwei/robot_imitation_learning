function best_q0_index = find_closet_joint_configuration(x_shoulder, xe_target, xw_target, xe_dataset, xw_dataset)
%% This function searches over the whole dataset and picks the closet joint configuration.


%% Parameters
num_samples = length(xe_dataset);
N_s_e = 10; N_e_w = 10; % interpolation points; don't set too large, or it could be really slow

%% Interpolate the points of the line crossing x_shoulder, xe_target and xw_target for later use
X_target_interp = zeros(3, N_s_e + N_e_w); % lose the middle point
for xi = 1 : 3 % computed only once
    X_target_interp(xi, :) = [linspace(x_shoulder(xi), xe_target(xi), N_s_e), linspace(xe_target(xi), xw_target(xi), N_e_w)];
end
X_target_interp = [X_target_interp(:, 1:N_s_e), X_target_interp(:, 2:end)]; % lose the duplicating middle points
% display the interpolation line
%{
figure;
plot3(X_target_interp(1, :), X_target_interp(2, :), X_target_interp(3, :), 'b.'); hold on;
plot3(x_shoulder(1), x_shoulder(2), x_shoulder(3), 'go');
plot3(xe_target(1), xe_target(2), xe_target(3), 'yo');
plot3(xw_target(1), xw_target(2), xw_target(3), 'ro');
grid on;
title('Target: shoulder --> elbow --> wrist ');
xlabel('x'); ylabel('y'); zlabel('z');
%}

%% Enumerate over the dataset, to find the best joint configuration
% iterate over different samples
for i_sample = 1 : num_samples
    
    % iterate over different time instance of a single sample
    for i_t = 1 : size(xe_dataset, 1)
        
        % obtain a specific position
        xe_demo = xe_dataset{i_sample}(i_t, :); %
        xw_demo = xw_dataset{i_sample}(i_t, :); % 1 x 3
        
        % interpolation line
        X_demo_interp = zeros(3, N_s_e + N_e_w);
        for xi = 1 : 3
            X_demo_interp(xi, :) = [linspace(x_shoulder(xi), xe_demo(xi), N_s_e), linspace(xe_demo(xi), xw_demo(xi), N_e_w)];
        end
        X_demo_interp = [X_demo_interp(:, 1:N_s_e), X_demo_interp(:, 2:end)]; % lose the duplicating middle point
        
        % display the interpolation line
        %{
        figure;
        plot3(X_demo_interp(1, :), X_demo_interp(2, :), X_demo_interp(3, :), 'b.'); hold on;
        plot3(x_shoulder(1), x_shoulder(2), x_shoulder(3), 'go');
        plot3(xe_demo(1), xe_demo(2), xe_demo(3), 'yo');
        plot3(xw_demo(1), xw_demo(2), xw_demo(3), 'ro');
        grid on;
        title('Demo: shoulder --> elbow --> wrist');
        xlabel('x'); ylabel('y'); zlabel('z');
        pause;
        %}
        
        % compute the Frechet distance between a sample and the goal
        [tmp_Frechet_dist, ~] = DiscreteFrechetDist(X_demo_interp', X_target_interp'); % use L2 norm as default
        if i_sample == 1 && i_t == 1
            min_Frechet_dist = tmp_Frechet_dist;
            best_q0_index_t = 1;
            best_q0_index_sample = 1;
        else
            if tmp_Frechet_dist < min_Frechet_dist
                % should return the sample index and the corresponding time t!!!
                min_Frechet_dist = tmp_Frechet_dist;
                best_q0_index_t = i_t;
                best_q0_index_sample = i_sample;
            end
        end
    end
end

best_q0_index = [best_q0_index_sample, best_q0_index_t];

end