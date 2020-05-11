%% Build datasets

num_resampled_points = 50;

%% Load data from mocap data file
%{
file_name = 'test_imi_data_YuMi.h5';
info = h5info(file_name);
% extract group_names and number of imitation data    
pattern = '[/-_]+\d'; % from / to _ followed by a digit number
group_name_dict = struct; % with no fields
for i = 1:size(info.Groups)
    str = info.Groups(i).Name;
    id = regexp(str, pattern);
    group_name = str(2:id-1);
    if isfield(group_name_dict, group_name)
        eval(['group_name_dict.', group_name, ' = ', 'group_name_dict.', group_name, ' + 1;']);
    else
        eval(['group_name_dict.', group_name, ' = 1;']);
    end
end
group_name_list = fieldnames(group_name_dict); % cell of group names

% iterate and apply transformations
for gp_id = 1 : size(group_name_list)
    num_imitation_data = eval(['group_name_dict.', group_name_list{gp_id}]);
%     original_trajs = [];
    similarity_preserved_trajs = [];
    similarity_destroyed_trajs = [];
    disp(['>>>> Group ', group_name_list{gp_id}, ' ...']);
    for traj_id = 1 : num_imitation_data
        %% Load original trajectory
        group_name = [group_name_list{gp_id}, '_', num2str(traj_id)];

        disp(['>>>> ---- processing ', group_name, ' ...']);

        l_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']); % 3 x N
        l_elbow_pos = h5read(file_name, ['/', group_name, '/l_elbow_pos']);
        l_wrist_pos = h5read(file_name, ['/', group_name, '/l_wrist_pos']);
        l_wrist_ori = h5read(file_name, ['/', group_name, '/l_wrist_ori']);  % 9 x N
        r_shoulder_pos = h5read(file_name, ['/', group_name, '/l_shoulder_pos']);
        r_elbow_pos = h5read(file_name, ['/', group_name, '/r_elbow_pos']);
        r_wrist_pos = h5read(file_name, ['/', group_name, '/r_wrist_pos']);
        r_wrist_ori = h5read(file_name, ['/', group_name, '/r_wrist_ori']);
        l_glove_angle = h5read(file_name, ['/', group_name, '/l_glove_angle']); % 14 x N
        r_glove_angle = h5read(file_name, ['/', group_name, '/r_glove_angle']);
        time = h5read(file_name, ['/', group_name, '/time']);
        length = size(l_wrist_pos, 2);
        % convert rotation matrices to euler angles
        % euler angles discontinuous due to singularity!!!
        l_wrist_euler = zeros(3, length);
        r_wrist_euler = zeros(3, length); % in radius
        l_wrist_quat = zeros(4, length);
        r_wrist_quat = zeros(4, length);
        for i = 1:length
            % left
            rotm = reshape(l_wrist_ori(:, i), 3, 3);
            rotm = rotm';
            eul = rotm2eul(rotm);
            quat = rotm2quat(rotm);
            l_wrist_euler(:, i) = eul';
            l_wrist_quat(:, i) = quat';
            % right
            rotm = reshape(r_wrist_ori(:, i), 3, 3);
            rotm = rotm';
            eul = rotm2eul(rotm);
            quat = rotm2quat(rotm);
            r_wrist_euler(:, i) = eul';
            r_wrist_quat(:, i) = quat';
        end
        
        original_traj = [l_wrist_pos; l_wrist_quat; l_elbow_pos; r_wrist_pos; r_wrist_quat; r_elbow_pos; l_glove_angle; r_glove_angle];
                        % 1,2,3;      4,5,6,7;      8, 9, 10;    11,12,13;    14,15,16,17;  18,19,20;      
        oritraj_id = sort(randperm(size(original_traj, 1)));
        quat_id = [4,5,6,7, 14,15,16,17];
        pos_id = [1,2,3, 8,9,10, 11,12,13, 18,19,20];
        wrist_id = [1,2,3, 11,12,13];
        elbow_id = [8,9,10, 18,19,20];
        glove_id = setdiff(setdiff(oritraj_id, quat_id), pos_id);
%         pos_angle_id = setdiff(oritraj_id, quat_id);

        % store the original samples
        original_traj_resampled = resample_normalize_flatten_traj(time, original_traj, num_resampled_points);
        similarity_preserved_trajs = [similarity_preserved_trajs; original_traj_resampled];
        
       %% Apply different transformations to obtain similarity_preserved and similarity_destroyed data
        % translate: no need
        
        % global scale: no need
        
        % resample: no need

        % rotation (on pos data)
        max_angle = 315 * pi/180; min_angle = 45 * pi/180;
        num_rot_samples = 9;
        rand_euler_angles = rand(num_rot_samples, 3) * (max_angle - min_angle) + min_angle;
        for n = 1 : num_rot_samples
            % initialize
            tmp_traj = original_traj;
            rotm = eul2rotm(rand_euler_angles(n, :));
            for pid = 1 : round(size(pos_id, 2)/3)
                tmp_traj_1d = tmp_traj(pos_id(3*pid-2 : 3*pid), :); % before rotation
                tmp_traj_1d = rotm * (tmp_traj_1d - repmat(tmp_traj_1d(:, floor(length/2)), 1, length)) + repmat(tmp_traj_1d(:, floor(length/2)), 1, length); % rotate around the trajectory mid point
                tmp_traj(pos_id(3*pid-2 : 3*pid), :) = tmp_traj_1d; % after rotation
            end
            % store the result
            tmp_traj_resampled = resample_normalize_flatten_traj(time, tmp_traj, num_resampled_points);
            similarity_destroyed_trajs = [similarity_destroyed_trajs; tmp_traj_resampled];
        end
        
        % add noise (to pos and angles)
        bnd_wrist = (max(tmp_traj(wrist_id, :), [], 2) - min(tmp_traj(wrist_id, :), [], 2)) / 50; % 1/50 of the range
        bnd_elbow = (max(tmp_traj(elbow_id, :), [], 2) - min(tmp_traj(elbow_id, :), [], 2)) / 50;
        bnd_glove = (max(tmp_traj(glove_id, :), [], 2) - min(tmp_traj(glove_id, :), [], 2)) / 100; %5; % +-5 degree range
        for n = 1 : 10
            % initialize
            tmp_traj = original_traj;
            % use uniformly distributed pseudorandom numbers
            n_wrist = rand(6, length) * 2 .* repmat(bnd_wrist, 1, length) - repmat(bnd_wrist, 1, length);
            n_elbow = rand(6, length) * 2 .* repmat(bnd_elbow, 1, length) - repmat(bnd_elbow, 1, length);
            n_glove = rand(28, length) * 2 .* repmat(bnd_glove, 1, length) - repmat(bnd_glove, 1, length);
            % left and right wrists
            tmp_traj(wrist_id, :) = tmp_traj(wrist_id, :) + n_wrist;
            % left and right elbows
            tmp_traj(elbow_id, :) = tmp_traj(elbow_id, :) + n_elbow;
            % left and right glove angles
            tmp_traj(glove_id, :) = tmp_traj(glove_id, :) + n_glove;
            
            % store result
            tmp_traj_resampled = resample_normalize_flatten_traj(time, tmp_traj, num_resampled_points);
            similarity_preserved_trajs = [similarity_preserved_trajs; tmp_traj_resampled];
        end
        
        % keypoint local shift (only on pos data for now; what about orientation ???)
        kp_id_list = extract_keypoints_func(l_wrist_pos, l_wrist_ori, l_elbow_pos, r_wrist_pos, r_wrist_ori, r_elbow_pos, l_glove_angle, r_glove_angle);
        num_kps = size(kp_id_list,2);
        for n = 1 : 10 % get 10 samples
            % initialize
            tmp_traj = original_traj;
            % display for debug - part 1
            %         figure;
            %         plot3(tmp_traj(wrist_id(1),:), tmp_traj(wrist_id(2),:), tmp_traj(wrist_id(3),:), 'b-'); hold on; grid on;
            %         plot3(tmp_traj(wrist_id(4),:), tmp_traj(wrist_id(5),:), tmp_traj(wrist_id(6),:), 'b-');
            %         plot3(tmp_traj(elbow_id(1),:), tmp_traj(elbow_id(2),:), tmp_traj(elbow_id(3),:), 'b-'); hold on; grid on;
            %         plot3(tmp_traj(elbow_id(4),:), tmp_traj(elbow_id(5),:), tmp_traj(elbow_id(6),:), 'b-');
            
            % set up range
            bnd_wrist = (max([tmp_traj(wrist_id(1:3), :), tmp_traj(wrist_id(4:6), :)], [], 2) ...
                - min([tmp_traj(wrist_id(1:3), :), tmp_traj(wrist_id(4:6), :)], [], 2)) / 40;
            bnd_elbow = (max([tmp_traj(elbow_id(1:3), :), tmp_traj(elbow_id(4:6), :)], [], 2) ...
                - min([tmp_traj(elbow_id(1:3), :), tmp_traj(elbow_id(4:6), :)], [], 2)) / 40;
            % set shift offset
            wrists_pos_offset = rand(3, num_kps) * 2 .* repmat(bnd_wrist, 1, num_kps) - repmat(bnd_wrist, 1, num_kps); % local shift range
            elbows_pos_offset = rand(3, num_kps) * 2 .* repmat(bnd_elbow, 1, num_kps) - repmat(bnd_elbow, 1, num_kps);
            % same offset for both arms' wrists
            tmp_traj(wrist_id(1:3), kp_id_list) = tmp_traj(wrist_id(1:3), kp_id_list) + wrists_pos_offset;
            tmp_traj(wrist_id(4:6), kp_id_list) = tmp_traj(wrist_id(4:6), kp_id_list) + wrists_pos_offset;
            % same offset for both arms' elbows
            tmp_traj(elbow_id(1:3), kp_id_list) = tmp_traj(elbow_id(1:3), kp_id_list) + elbows_pos_offset;
            tmp_traj(elbow_id(4:6), kp_id_list) = tmp_traj(elbow_id(4:6), kp_id_list) + elbows_pos_offset;
            
            % display for debug - part 2
            %         plot3(tmp_traj(wrist_id(1),:), tmp_traj(wrist_id(2),:), tmp_traj(wrist_id(3),:), 'r-');
            %         plot3(tmp_traj(wrist_id(4),:), tmp_traj(wrist_id(5),:), tmp_traj(wrist_id(6),:), 'r-');
            %         plot3(tmp_traj(elbow_id(1),:), tmp_traj(elbow_id(2),:), tmp_traj(elbow_id(3),:), 'r-');
            %         plot3(tmp_traj(elbow_id(4),:), tmp_traj(elbow_id(5),:), tmp_traj(elbow_id(6),:), 'r-');
            
            % store the result
            tmp_traj_resampled = resample_normalize_flatten_traj(time, tmp_traj, num_resampled_points);
            similarity_preserved_trajs = [similarity_preserved_trajs; tmp_traj_resampled];
        end
        
        disp('done.');
        
    end
    
    %% Store similarity_preserved and similarity_destroyed data to file
    new_file_name = 'test_imi_data_YuMi_similarity_transformed.h5';
%     h5create(new_file_name, ['/', group_name_list{gp_id}, '/original_trajs'], size(original_trajs));
    h5create(new_file_name, ['/', group_name_list{gp_id}, '/similarity_preserved_trajs'], size(similarity_preserved_trajs));
    h5create(new_file_name, ['/', group_name_list{gp_id}, '/similarity_destroyed_trajs'], size(similarity_destroyed_trajs));

%     h5write(new_file_name, ['/', group_name_list{gp_id}, '/original_trajs'], original_trajs);
    h5write(new_file_name, ['/', group_name_list{gp_id}, '/similarity_preserved_trajs'], similarity_preserved_trajs);
    h5write(new_file_name, ['/', group_name_list{gp_id}, '/similarity_destroyed_trajs'], similarity_destroyed_trajs);
    
end
%}


%% Construct training samples - pairs
%
% Load raw data file
new_file_name = 'test_imi_data_YuMi_similarity_transformed.h5';
info = h5info(new_file_name);
num_groups = size(info.Groups, 1);

% initialize
similar_class = [];
dissimilar_class = [];
% construct labelled dataset for training, from two ways of construction    
for i = 1 : num_groups
    disp(['Processing Group ', num2str(i), ' ...']);
    % get ready
    other_groups = setdiff(randperm(num_groups), i);
    cur_group_similar = h5read(new_file_name, [info.Groups(i).Name, '/similarity_preserved_trajs']);
    cur_group_dissimilar = h5read(new_file_name, [info.Groups(i).Name, '/similarity_destroyed_trajs']); 
    num_similar = size(cur_group_similar, 1);
    num_dissimilar = size(cur_group_dissimilar, 1);
    
    % get samples of similar class from within the current group
    num_pair = 20; % get 20 samples 
    rand_id = [randperm(num_similar, num_pair); randperm(num_similar, num_pair)];
    similar_class = [similar_class; [cur_group_similar(rand_id(1,:), :), cur_group_similar(rand_id(2,:), :)] ];

    % get samples of dissimilar class from within the current group   
    num_pair = 10; % get 20 samples 
    rand_id = [randperm(num_similar, num_pair); randperm(num_dissimilar, num_pair)];
    dissimilar_class = [dissimilar_class; [cur_group_similar(rand_id(1,:), :), cur_group_dissimilar(rand_id(2,:), :)] ];
    
    % get samples of dissimilar class from different groups    
    num_gp_pair = 5;
    num_pair_per_group = 2; % yields num_gp_pair x num_pair_per_group pairs of dissimilar data  
    gp_id = randi(num_groups-1, num_gp_pair, 1); % randomly choose group to access
    for j = 1 : num_gp_pair
        target_gp_id = gp_id(j); % ID of the group to access
        target_group_similar = h5read(new_file_name, [info.Groups(target_gp_id).Name, '/similarity_preserved_trajs']); 
        num_target_similar = size(target_group_similar, 1);
        % get pair
        rand_id = [randperm(num_similar, num_pair_per_group); randperm(num_target_similar, num_pair_per_group)];
        dissimilar_class = [dissimilar_class; [cur_group_similar(rand_id(1,:), :), target_group_similar(rand_id(2,:), :)] ];
    end
    
    disp('done');
    
end

% store the training datasets
train_file_name = 'test_imi_data_YuMi_training_dataset.h5';
h5create(train_file_name, '/similar', size(similar_class));
h5create(train_file_name, '/dissimilar', size(dissimilar_class));

h5write(train_file_name, '/similar', similar_class);
h5write(train_file_name, '/dissimilar', dissimilar_class);


% further divide into x_train and x_test
num_similar_samples = size(similar_class, 1);
num_dissimilar_samples = size(dissimilar_class, 1);
y_similar = ones(num_similar_samples, 1) * 1; % labels
y_dissimilar = zeros(num_dissimilar_samples, 1) * 0;
% 3/4 of samples for training (ratio is 1:3)
similar_train_id = sort(randperm(num_similar_samples, round(num_similar_samples * 3 / 4)));
similar_test_id = setdiff(randperm(num_similar_samples), similar_train_id);
dissimilar_train_id = sort(randperm(num_dissimilar_samples, round(num_dissimilar_samples * 3 / 4)));
dissimilar_test_id = setdiff(randperm(num_dissimilar_samples), dissimilar_train_id);
% construct training samples
x_train = [similar_class(similar_train_id, :); dissimilar_class(dissimilar_train_id, :)];
x_test = [similar_class(similar_test_id, :); dissimilar_class(dissimilar_test_id, :)];
y_train = [y_similar(similar_train_id, :); y_dissimilar(dissimilar_train_id, :)];
y_test = [y_similar(similar_test_id, :); y_dissimilar(dissimilar_test_id, :)];
% randomize the order
id_train = randperm(size(x_train, 1));
id_test = randperm(size(x_test, 1));
x_train_rand = x_train(id_train, :);
y_train_rand = y_train(id_train);
x_test_rand = x_test(id_test, :);
y_test_rand = y_test(id_test);

train_file_name = 'test_imi_data_YuMi_training_dataset.h5';
h5create(train_file_name, '/x_train', size(x_train_rand));
h5create(train_file_name, '/y_train', size(y_train_rand));
h5create(train_file_name, '/x_test', size(x_test_rand));
h5create(train_file_name, '/y_test', size(y_test_rand));

h5write(train_file_name, '/x_train', x_train_rand);
h5write(train_file_name, '/y_train', y_train_rand);
h5write(train_file_name, '/x_test', x_test_rand);
h5write(train_file_name, '/y_test', y_test_rand);


%}
