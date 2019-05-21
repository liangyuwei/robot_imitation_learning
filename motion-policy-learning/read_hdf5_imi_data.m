function [Q_l, Q_r] = read_hdf5_imi_data(file_name)
%% This function reads the imitation dataset and returns needed data.

%% Read .h5 file
info = h5info(file_name);
n_samples = length(info.Groups)/2;
Q_l = cell(n_samples, 2); % initialization, inconsistent time ==> must use cell  
Q_r = cell(n_samples, 2);
for i = 1:n_samples
   % Access the ith sample 
   %info.Groups(i).Datasets(2) % the 2nd data item is 'pos'
   %info.Groups(i).Datasets(3) % the 3rd data item is 'time_from_start'
   data_l = h5read(file_name, ['/traj_pair_l_', num2str(i), '/pos']); % 6 x n_data_points
   time_l = h5read(file_name, ['/traj_pair_l_', num2str(i), '/time_from_start']); % n_data_points x 1
   Q_l{i, 1} = data_l';
   Q_l{i, 2} = time_l;
   data_r = h5read(file_name, ['/traj_pair_r_', num2str(i), '/pos']); % 6 x n_data_points
   time_r = h5read(file_name, ['/traj_pair_r_', num2str(i), '/time_from_start']); % n_data_points x 1
   Q_r{i, 1} = data_r';
   Q_r{i, 2} = time_r;
end



end