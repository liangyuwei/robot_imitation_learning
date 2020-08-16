

ori_file_name = '../motion-retargeting/test_imi_data_YuMi.h5';

group_name = 'gun_2';

l_glove_angle = h5read(ori_file_name, ['/', group_name, '/l_glove_angle']);
% l_glove_angle_resampled = h5read(ori_file_name, ['/', group_name, '/l_glove_angle_resampled']);
                                                                                                 
r_glove_angle = h5read(ori_file_name, ['/', group_name, '/r_glove_angle']);
% r_glove_angle_resampled = h5read(ori_file_name, ['/', group_name, '/r_glove_angle_resampled']);


plot_joints_comp_subplots(l_glove_angle(1:7, :), r_glove_angle(1:7, :), 'Hand joints', 'l\_glove\_angle', 'r\_glove\_angle');


plot_joints_comp_subplots(l_glove_angle(8:end, :), r_glove_angle(7:end, :), 'Hand joints', 'l\_glove\_angle', 'r\_glove\_angle');



n = 4;
figure; 
plot(l_glove_angle(n, :), 'b-'); hold on; grid on; 
plot(l_glove_angle_resampled(n, :), 'r-');
title('l\_glove\_angle');

figure; 
plot(r_glove_angle(n, :), 'b-'); hold on; grid on; 
plot(r_glove_angle_resampled(n, :), 'r-');
title('r\_glove\_angle');






