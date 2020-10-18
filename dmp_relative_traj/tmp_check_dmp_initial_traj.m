figure;

plot3(l_wrist_pos_human(1,:), l_wrist_pos_human(2,:), l_wrist_pos_human(3,:), 'b-'); hold on; grid on;

plot3(y_l_wrist_initial(1, :), y_l_wrist_initial(2, :), y_l_wrist_initial(3, :), 'r--'); 



figure;
plot3(l_elbow_pos_human(1,:), l_elbow_pos_human(2,:), l_elbow_pos_human(3,:), 'b-'); hold on; grid on;
plot3(y_l_elbow_initial(1, :), y_l_elbow_initial(2, :), y_l_elbow_initial(3, :), 'r--'); 

plot3(r_elbow_pos_human(1,:), r_elbow_pos_human(2,:), r_elbow_pos_human(3,:), 'b-'); hold on; grid on;
plot3(y_r_elbow_initial(1, :), y_r_elbow_initial(2, :), y_r_elbow_initial(3, :), 'r--'); 



for i = 1 : 3
   l_elbow_pos_human(i, :) = mapminmax(l_elbow_pos_human(i, :), 0, 1);
   r_elbow_pos_human(i, :) = mapminmax(r_elbow_pos_human(i, :), 0, 1);
   y_l_elbow_initial(i, :) = mapminmax(y_l_elbow_initial(i, :), 0, 1);
   y_r_elbow_initial(i, :) = mapminmax(y_r_elbow_initial(i, :), 0, 1);
end


figure;
subplot(3, 1, 1);
plot(l_elbow_pos_human(1,:), 'b-'); hold on; grid on;
plot(y_l_elbow_initial(1,:), 'r--'); 

subplot(3, 1, 2);
plot(l_elbow_pos_human(2,:), 'b-'); hold on; grid on;
plot(y_l_elbow_initial(2,:), 'r--'); 

subplot(3, 1, 3);
plot(l_elbow_pos_human(3,:), 'b-'); hold on; grid on;
plot(y_l_elbow_initial(3,:), 'r--'); 


figure;
subplot(3, 1, 1);
plot(r_elbow_pos_human(1,:), 'b-'); hold on; grid on;
plot(y_r_elbow_initial(1,:), 'r--'); 

subplot(3, 1, 2);
plot(r_elbow_pos_human(2,:), 'b-'); hold on; grid on;
plot(y_r_elbow_initial(2,:), 'r--'); 

subplot(3, 1, 3);
plot(r_elbow_pos_human(3,:), 'b-'); hold on; grid on;
plot(y_r_elbow_initial(3,:), 'r--'); 




figure;
subplot(3, 1, 1);
plot(x(1,:), 'b-'); hold on; grid on;
plot(y(1,:), 'r--'); 

subplot(3, 1, 2);
plot(x(2,:), 'b-'); hold on; grid on;
plot(y(2,:), 'r--'); 

subplot(3, 1, 3);
plot(x(3,:), 'b-'); hold on; grid on;
plot(y(3,:), 'r--'); 


