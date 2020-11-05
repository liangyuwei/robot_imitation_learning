function [angle_calib_l, angle_calib_r] = calib_finger(angle_l, elec_l, angle_recorded_l, elec_calib_l, angle_r, elec_r, angle_recorded_r, elec_calib_r, display, joint_flag)
%% Calibrate finger joint data using paired data of angle and electricity data.
% Input:
%   angle_lr - angle data, 1 x N, for calibration.
%   elec_lr - electricity signal data, 1 x N, for calibration.
%   elec_calib_lr - electricity signal data to calibrate.
%   angle_recorded_lr - recorded angle data given by dataglove SDK, which is actually linearly mapped.      
% Output:
%   angle_calib_lr - calibrated angle data.


%% Fit polynomials for calibration
n = 2; %4; %3; %2; 
[p_l, s_l] = polyfit(double(elec_l), double(angle_l), n);
[p_r, s_r] = polyfit(double(elec_r), double(angle_r), n);


% calibrate the raw data
angle_calib_l = polyval(p_l, double(elec_calib_l));
angle_calib_r = polyval(p_r, double(elec_calib_r));
% angle_calib_l = interp1(double(elec_l), double(angle_l), double(max(min(elec_calib_l, max(elec_l)), min(elec_r))), 'linear');
% angle_calib_r = interp1(double(elec_r), double(angle_r), double(max(min(elec_calib_r, max(elec_r)), min(elec_r))), 'linear');
% angle_calib_l = spline(double(elec_l), double(angle_l), double(elec_calib_l));
% angle_calib_r = spline(double(elec_r), double(angle_r), double(elec_calib_r));


% display the results for comparison
num_points = size(elec_calib_l, 2);
if (display)
    
    figure;
    p1 = plot(1:num_points, angle_recorded_l, 'b-'); hold on; grid on;
    p2 = plot(1:num_points, angle_recorded_r, 'r-');
    xlabel('Points'); ylabel('Angle');
    title(['Linearly Mapped Left and Right Index ', joint_flag, ' Joint']);
    legend([p1(1), p2(1)], ['Left Index ', joint_flag], ['Right Index ', joint_flag], 'Location', 'NorthEastOutside');
    
    % calibrated
    figure;
    p1 = plot(1:num_points, angle_calib_l, 'b-'); hold on; grid on;
    p2 = plot(1:num_points, angle_calib_r, 'r-');
    xlabel('Points'); ylabel('Angle');
    title(['Calibrated Left and Right Index ', joint_flag, ' Joint']);
    legend([p1(1), p2(1)], ['Left Index ', joint_flag], ['Right Index ', joint_flag], 'Location', 'NorthEastOutside');
    
end




end
