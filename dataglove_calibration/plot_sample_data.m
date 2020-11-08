function plot_sample_data(elec_90, elec_60, elec_30, elec_0, display_tag)
%% 


figure;

subplot(2, 2, 1);
plot(elec_90, 'b.'); hold on; grid on;
plot(1:size(elec_90, 2), mean(elec_90)*ones(size(elec_90)), 'r-');
plot(1:size(elec_90, 2), double(median(elec_90))*ones(size(elec_90)), 'g--');
title([display_tag, ' electrical signal under 90 deg']);

subplot(2, 2, 2);
plot(elec_60, 'b.'); hold on; grid on;
plot(1:size(elec_60, 2), mean(elec_60)*ones(size(elec_60)), 'r-');
plot(1:size(elec_60, 2), double(median(elec_60))*ones(size(elec_60)), 'g--');
title([display_tag, ' electrical signal under 60 deg']);
subplot(2, 2, 3);
plot(elec_30, 'b.'); hold on; grid on;
plot(1:size(elec_30, 2), mean(elec_30)*ones(size(elec_30)), 'r-');
plot(1:size(elec_30, 2), double(median(elec_30))*ones(size(elec_30)), 'g--');
title([display_tag, ' electrical signal under 30 deg']);
subplot(2, 2, 4);
plot(elec_0, 'b.'); hold on; grid on;
plot(1:size(elec_0, 2), mean(elec_0)*ones(size(elec_0)), 'r-');
plot(1:size(elec_0, 2), double(median(elec_0))*ones(size(elec_0)), 'g--');
title([display_tag, ' electrical signal under 0 deg']);



end