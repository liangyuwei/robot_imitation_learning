





[le_human, le_ours, p1] = DTW(l_elbow_pos_human, actual_l_elbow_pos_traj);
% l_elbow_pos_human(:, p1(1,:)) == le_human !!!

dist_ours = dtwdist(l_elbow_pos_human, actual_l_elbow_pos_traj)


[le_human2, le_hujin, p2] = DTW(l_elbow_pos_human, actual_l_elbow_pos_traj_hujin);

dist_hujin = dtwdist(l_elbow_pos_human, actual_l_elbow_pos_traj_hujin)


figure;
subplot(3, 1, 1);
plot(le_human(1,:), 'b-'); hold on; grid on;
plot(le_human2(1,:), 'r--'); 
plot(le_ours(1, :), 'g--');
plot(le_hujin(1, :), 'm--');

subplot(3, 1, 2);
plot(le_human(2,:), 'b-'); hold on; grid on;
plot(le_human2(2,:), 'r--'); 
plot(le_ours(2, :), 'g--');
plot(le_hujin(2, :), 'm--');

subplot(3, 1, 3);
plot(le_human(3,:), 'b-'); hold on; grid on;
plot(le_human2(3,:), 'r--'); 
plot(le_ours(3, :), 'g--');
plot(le_hujin(3, :), 'm--');
















