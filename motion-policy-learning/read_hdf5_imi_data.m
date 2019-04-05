function Q = read_hdf5_imi_data(file_name)
%% This function reads the imitation dataset and returns needed data.

%% Read .h5 file
info = h5info('imi_joint_traj_dataset.h5');
n_samples = length(info.Groups);
Q = cell(n_samples, 2); % initialization, inconsistent time ==> must use cell  
for i = 1:n_samples
   % Access the ith sample 
   %info.Groups(i).Datasets(2) % the 2nd data item is 'pos'
   %info.Groups(i).Datasets(3) % the 3rd data item is 'time_from_start'
   data = h5read('imi_joint_traj_dataset.h5', ['/imi_path_', num2str(i), '/pos']); % 6 x n_data_points
   time = h5read('imi_joint_traj_dataset.h5', ['/imi_path_', num2str(i), '/time_from_start']); % n_data_points x 1
   Q{i, 1} = data';
   Q{i, 2} = time;
end



end