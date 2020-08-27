function mapped_data = linear_map(input_data, min_x, max_x, min_x_hat, max_x_hat)
%% This function linearly maps the input electrical signal according to the minimum and maximum values, as is applied by WiseGlove SKD.
% Input:
%   input_data - the input electrical signal data, should be of size 1 x N, with N being the length.   
%   min_x, max_x - the minimum and maximum electric signal values.
%   min_x_hat, miax_x_hat - the minimum and maximum angle values.
% Output:
%   mapped_data - the linearly mapped (and clamped) data, should be pf size 1 x N, with N being the length.   


%% Convert int datatype to double
input_data = double(input_data);
min_x = double(min_x);
max_x = double(max_x);
min_x_hat = double(min_x_hat);
max_x_hat = double(max_x_hat);


%% Linear mapping (clamp before mapping)
% clampping version
mapped_data = max(min((input_data - min_x) / (max_x - min_x), 1.0), 0.0) * (max_x_hat - min_x_hat) + min_x_hat;
% no clampping version (might yields unexpected results)
% mapped_data = (input_data - min_x) / (max_x - min_x) * (max_x_hat - min_x_hat) + min_x_hat;


end

