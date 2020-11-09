function [angle_calib_l, angle_calib_r] = calib_finger(angle_l, elec_l, angle_recorded_l, elec_calib_l, ...
                                                       angle_r, elec_r, angle_recorded_r, elec_calib_r, ...
                                                       l_angle_min, l_angle_max, ...
                                                       r_angle_min, r_angle_max, ...
                                                       display, joint_flag)
%% Calibrate finger joint data using paired data of angle and electricity data.
% Input:
%   angle_lr - angle data, 1 x N, for calibration.
%   elec_lr - electricity signal data, 1 x N, for calibration.
%   elec_calib_lr - electricity signal data to calibrate.
%   angle_recorded_lr - recorded angle data given by dataglove SDK, which is actually linearly mapped.      
% Output:
%   angle_calib_lr - calibrated angle data.


%% Fit polynomials for calibration
% choose order of polynomial
% n = 3;%1; %2; %4; %3; %2; 
% [p_l, s_l] = polyfit(double(elec_l), double(angle_l), 4); %n);
% [p_r, s_r] = polyfit(double(elec_r), double(angle_r), 2); %n);

% Least Mean Squares regression
[p_l, s_l] = polyfit(double(angle_l), double(elec_l), 1); 
a = p_l(1); b = p_l(2);
p_l(1) = 1 / a; p_l(2) = -b / a; % convert to angle = (elec - b) / a
[p_r, s_r] = polyfit(double(angle_r), double(elec_r), 1); 
a = p_r(1); b = p_r(2);
p_r(1) = 1 / a; p_r(2) = -b / a; % convert to angle = (elec - b) / a


% calibrate the raw data
angle_calib_l = polyval(p_l, double(elec_calib_l));
angle_calib_r = polyval(p_r, double(elec_calib_r));
% angle_calib_l = interp1(double(elec_l), double(angle_l), double(max(min(elec_calib_l, max(elec_l)), min(elec_r))), 'linear');
% angle_calib_r = interp1(double(elec_r), double(angle_r), double(max(min(elec_calib_r, max(elec_r)), min(elec_r))), 'linear');
% angle_calib_l = spline(double(elec_l), double(angle_l), double(elec_calib_l));
% angle_calib_r = spline(double(elec_r), double(angle_r), double(elec_calib_r));


% post-processing, clampping resultant angles to be within range
angle_calib_l = max(min(angle_calib_l, l_angle_max), l_angle_min);
angle_calib_r = max(min(angle_calib_r, r_angle_max), r_angle_min); 


% display the results for comparison
num_points = size(elec_calib_l, 2);
if (display)
    % linearly mapped (provided by Wiseglove SDK)
    figure;
    p1 = plot(1:num_points, angle_recorded_l, 'b-'); hold on; grid on;
    p2 = plot(1:num_points, angle_recorded_r, 'r-');
    xlabel('Points'); ylabel('Angle');
    title(['Linearly Mapped Left and Right Index ', joint_flag, ' Joint']);
    legend([p1(1), p2(1)], ['Left Index ', joint_flag], ['Right Index ', joint_flag], 'Location', 'NorthEastOutside');
    
%     angle_recorded_lr_norm = mapminmax([angle_recorded_l, angle_recorded_r], 0, 1);
%     angle_recorded_l_norm = angle_recorded_lr_norm(1 : length(angle_recorded_l));
%     angle_recorded_r_norm = angle_recorded_lr_norm(length(angle_recorded_l)+1 : end);
%     figure;
%     p1 = plot(1:num_points, angle_recorded_l_norm, 'b-'); hold on; grid on;
%     p2 = plot(1:num_points, angle_recorded_r_norm, 'r-');
%     xlabel('Points'); ylabel('Angle');
%     title(['Linearly Mapped Left and Right Index (normalized) ', joint_flag, ' Joint']);
%     legend([p1(1), p2(1)], ['Left Index ', joint_flag], ['Right Index ', joint_flag], 'Location', 'NorthEastOutside');
    
    
    % calibrated
    figure;
    p1 = plot(1:num_points, angle_calib_l, 'b-'); hold on; grid on;
    p2 = plot(1:num_points, angle_calib_r, 'r-');
    xlabel('Points'); ylabel('Angle');
    title(['Calibrated Left and Right Index ', joint_flag, ' Joint']);
    legend([p1(1), p2(1)], ['Left Index ', joint_flag], ['Right Index ', joint_flag], 'Location', 'NorthEastOutside');
    
%     angle_calib_lr_norm = mapminmax([angle_calib_l, angle_calib_r], 0, 1);
%     angle_calib_l_norm = angle_calib_lr_norm(1 : length(angle_calib_l));
%     angle_calib_r_norm = angle_calib_lr_norm(length(angle_calib_l)+1 : end);
%     figure;
%     p1 = plot(1:num_points, angle_calib_l_norm, 'b-'); hold on; grid on;
%     p2 = plot(1:num_points, angle_calib_r_norm, 'r-');
%     xlabel('Points'); ylabel('Angle');
%     title(['Calibrated Left and Right Index (normalized) ', joint_flag, ' Joint']);
%     legend([p1(1), p2(1)], ['Left Index ', joint_flag], ['Right Index ', joint_flag], 'Location', 'NorthEastOutside');
    
    
    % error comparison
    figure;
    p1 = plot(1:num_points, abs(angle_recorded_l - angle_recorded_r), 'b-'); hold on; grid on;
    p2 = plot(1:num_points, abs(angle_calib_l - angle_calib_r), 'r-'); 
    xlabel('Points'); ylabel('Angle');
    title(['Comparison of Left-Right difference in ', joint_flag, ' Joint']);
    legend([p1(1), p2(1)], ['Linearly Mapped ', joint_flag], ['Calibrated ', joint_flag], 'Location', 'NorthEastOutside');
    
end




end
