%% Build datasets

num_resampled_points = 100;

%% Load data from mocap data file
%
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
for gp_id = 5%1 : size(group_name_list, 1)
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
        % display for debug
        %{
        idxx = [11,12,13]; %[1,2,3];
        original_traj_display = zeros(size(idxx, 2), num_resampled_points);
        for id = 1 : num_resampled_points
            original_traj_display(1, id) = original_traj_resampled(idxx(1)+DOF*(id-1));
            original_traj_display(2, id) = original_traj_resampled(idxx(2)+DOF*(id-1));
            original_traj_display(3, id) = original_traj_resampled(idxx(3)+DOF*(id-1));
        end        
        figure;
        plot3(original_traj_display(1, :), original_traj_display(2, :), original_traj_display(3, :), 'b-'); hold on; grid on;
        title('Original and transformed trajectories');
        xlabel('x'); ylabel('y'); zlabel('z'); 
        %}
        
        
        % store the resampled_normalized_flattened version of the original trajectory in h5 file    
        h5create(file_name, ['/', group_name, '/resampled_normalized_flattened_oritraj'], size(original_traj_resampled));
        h5write(file_name, ['/', group_name, '/resampled_normalized_flattened_oritraj'], original_traj_resampled);
        
        
       %% Apply different transformations to obtain similarity_preserved and similarity_destroyed data
        % translate: no need
        
        % global scale: no need
        
        % resample: no need

        % rotation (on pos data)
        max_angle = 315 * pi/180; min_angle = 45 * pi/180;
        num_rot_samples = 64;
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
            % display for presentation
            %{
            tmp_traj_display = zeros(size(idxx, 2), num_resampled_points);
            for id = 1 : num_resampled_points
                tmp_traj_display(1, id) = tmp_traj_resampled(idxx(1)+DOF*(id-1));
                tmp_traj_display(2, id) = tmp_traj_resampled(idxx(2)+DOF*(id-1));
                tmp_traj_display(3, id) = tmp_traj_resampled(idxx(3)+DOF*(id-1));
            end
            plot3(tmp_traj_display(1, :), tmp_traj_display(2, :), tmp_traj_display(3, :), 'r-'); hold on; grid on;
            %}
        end
        
        % add noise (to pos and angles)
        bnd_wrist = (max(tmp_traj(wrist_id, :), [], 2) - min(tmp_traj(wrist_id, :), [], 2)) / 50; % 1/50 of the range
        bnd_elbow = (max(tmp_traj(elbow_id, :), [], 2) - min(tmp_traj(elbow_id, :), [], 2)) / 50;
        bnd_glove = (max(tmp_traj(glove_id, :), [], 2) - min(tmp_traj(glove_id, :), [], 2)) / 100; %5; % +-5 degree range
        for n = 1 : 50
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
            % display for presentation
            %{
            tmp_traj_display = zeros(size(idxx, 2), num_resampled_points);
            for id = 1 : num_resampled_points
                tmp_traj_display(1, id) = tmp_traj_resampled(idxx(1)+DOF*(id-1));
                tmp_traj_display(2, id) = tmp_traj_resampled(idxx(2)+DOF*(id-1));
                tmp_traj_display(3, id) = tmp_traj_resampled(idxx(3)+DOF*(id-1));
            end
            plot3(tmp_traj_display(1, :), tmp_traj_display(2, :), tmp_traj_display(3, :), 'g-'); hold on; grid on;
            %}
        end
        
        % keypoint local shift (only on pos data for now; what about orientation ???)
        kp_id_list = extract_keypoints_func(l_wrist_pos, l_wrist_ori, l_elbow_pos, r_wrist_pos, r_wrist_ori, r_elbow_pos, l_glove_angle, r_glove_angle);
        num_kps = size(kp_id_list,2);
        for n = 1 : 50 
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
                - min([tmp_traj(wrist_id(1:3), :), tmp_traj(wrist_id(4:6), :)], [], 2)) / 40; % noise within 1/40 of the magnitude
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
            
            % display for presentation
            %{
            tmp_traj_display = zeros(size(idxx, 2), num_resampled_points);
            for id = 1 : num_resampled_points
                tmp_traj_display(1, id) = tmp_traj_resampled(idxx(1)+DOF*(id-1));
                tmp_traj_display(2, id) = tmp_traj_resampled(idxx(2)+DOF*(id-1));
                tmp_traj_display(3, id) = tmp_traj_resampled(idxx(3)+DOF*(id-1));
            end
            plot3(tmp_traj_display(1, :), tmp_traj_display(2, :), tmp_traj_display(3, :), 'g-'); hold on; grid on;
            %}
                        
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

% settings
num_intra_group_similar = 100;
num_intra_group_dissimilar = 50;
num_inter_group_dissimilar = 50;
min_num_similar = Inf; min_num_dissimilar = Inf;
for n = 1:num_groups
    % obtain the smallest number 
    if strcmp(info.Groups(n).Datasets(1).Name, 'similarity_destroyed_trajs')
        num_dissimilar = info.Groups(n).Datasets(1).Dataspace.Size(1);
        num_similar = info.Groups(n).Datasets(2).Dataspace.Size(1);
    else
        num_dissimilar = info.Groups(n).Datasets(2).Dataspace.Size(1);
        num_similar = info.Groups(n).Datasets(1).Dataspace.Size(1);
    end
    % store the minimum number
    if num_dissimilar < min_num_dissimilar
        min_num_dissimilar = num_dissimilar;
    end
    if num_similar < min_num_similar
        min_num_similar = num_similar;
    end
end
assert(num_intra_group_similar * 2 + num_intra_group_dissimilar + num_inter_group_dissimilar ...
        < min_num_similar, 'Samples not enough for unique selection of similar pairs!!!');
assert(num_intra_group_dissimilar < min_num_dissimilar, 'Samples not enough for unique selection of dissimilar pairs!!!');

% split the groups(vocabularies) into train set and test set (isolation)
num_train_groups = round(num_groups * 3/4); % train set : test set, 1:3
num_test_groups = num_groups - num_train_groups;
train_group_id = randperm(num_groups, num_train_groups); % randomly select groups to be inside train set or test set
test_group_id = setdiff(1:num_groups, train_group_id);

% set group id pair (if num_groups is odd, one group would appear twice)
if mod(num_train_groups, 2) ~= 0
    % odd number of 
    assert(num_intra_group_similar * 2 + num_intra_group_dissimilar + ...
           num_inter_group_dissimilar * 2 < min_num_similar, ...
           'Similar samples may not be enough for unique selection of inter-group dissimilar pairs');
    % choose one group to be used twice for inter-group sample selection
    double_effort_train_group_id = train_group_id(randperm(num_train_groups, 1)); % randomly select one group ID inside train group
end
if mod(num_test_groups, 2) ~= 0
    % odd number of 
    assert(num_intra_group_similar * 2 + num_intra_group_dissimilar + ...
           num_inter_group_dissimilar * 2 < min_num_similar, ...
           'Similar samples may not be enough for unique selection of inter-group dissimilar pairs');
    % choose one group to be used twice for inter-group sample selection
    double_effort_test_group_id = test_group_id(randperm(num_test_groups, 1)); % randomly select one group ID inside test group
end

found = false;
while ~found
    unique_id = randperm(num_train_groups, num_train_groups); % find index to num_train_group_id
    inter_train_group_pair = [train_group_id(unique_id(1:floor(num_train_groups/2))); ...
                              train_group_id(unique_id(floor(num_train_groups/2)+1 : floor(num_train_groups/2)*2))];
    if mod(num_train_groups, 2) ~= 0
        % odd number
        if train_group_id(unique_id(end)) == double_effort_train_group_id
            continue;
        end
        inter_train_group_pair = [inter_train_group_pair, [double_effort_train_group_id; train_group_id(unique_id(end))]];        
    end
    found = true; % in case the number is even...
end

found = false;
while ~found
    unique_id = randperm(num_test_groups, num_test_groups); % find index to num_train_group_id
    inter_test_group_pair = [test_group_id(unique_id(1:floor(num_test_groups/2))); ...
                             test_group_id(unique_id(floor(num_test_groups/2)+1 : floor(num_test_groups/2)*2))];
    if mod(num_test_groups, 2) ~= 0
        % odd number
        if test_group_id(unique_id(end)) == double_effort_test_group_id
            continue;
        end
        inter_test_group_pair = [inter_test_group_pair, [double_effort_test_group_id; test_group_id(unique_id(end))]];
    end
    found = true; % in case the number is even...
end

% initialize
train_similar_class = [];
train_dissimilar_class = [];
test_similar_class = [];
test_dissimilar_class = [];

group_similar_samples_id = zeros(num_groups, num_intra_group_similar*2+num_intra_group_dissimilar+num_inter_group_dissimilar);
group_dissimilar_samples_id = zeros(num_groups, num_intra_group_dissimilar);

% construct labelled dataset for training, from two ways of construction   
for i = 1 : num_groups
    disp(['Processing Group ', num2str(i), ' ...']);
    % Get ready
    other_groups = setdiff(randperm(num_groups), i);
    cur_group_similar = h5read(new_file_name, [info.Groups(i).Name, '/similarity_preserved_trajs']);
    cur_group_dissimilar = h5read(new_file_name, [info.Groups(i).Name, '/similarity_destroyed_trajs']); 
    num_similar = size(cur_group_similar, 1);
    num_dissimilar = size(cur_group_dissimilar, 1);

    % set up IDs
    group_similar_samples_id(i, :) = randperm(num_similar, size(group_similar_samples_id, 2));
    group_dissimilar_samples_id(i, :) = randperm(num_dissimilar, size(group_dissimilar_samples_id, 2));
    if mod(num_train_groups, 2) ~= 0
        % odd number
        if i == double_effort_train_group_id
            rest_id_all = setdiff(1:num_similar, group_similar_samples_id(i, :));
            rest_train_id = rest_id_all(randperm(size(rest_id_all,2), num_inter_group_dissimilar));
        end
    end
    if mod(num_test_groups, 2) ~= 0
        % odd number
        if i == double_effort_test_group_id
            rest_id_all = setdiff(1:num_similar, group_similar_samples_id(i, :));
            rest_test_id = rest_id_all(randperm(size(rest_id_all,2), num_inter_group_dissimilar));
        end
    end
    
    % 1- Get samples of similar class from within the current group
    rand_id = [group_similar_samples_id(i, 1 : num_intra_group_similar); ...
               group_similar_samples_id(i, num_intra_group_similar+1 : 2*num_intra_group_similar)];
    if ~isempty(find(i == train_group_id, 1))
        % if i is inside train_group_id, i.e. the current group belongs to train set                 
        train_similar_class = [train_similar_class; [cur_group_similar(rand_id(1,:), :), cur_group_similar(rand_id(2,:), :)] ];
    else        
        test_similar_class = [test_similar_class; [cur_group_similar(rand_id(1,:), :), cur_group_similar(rand_id(2,:), :)] ];
    end
    
    % 2 - Get samples of dissimilar class from within the current group   
    rand_id = [group_similar_samples_id(i, 2*num_intra_group_similar+1 : 2*num_intra_group_similar+num_intra_group_dissimilar); ...
               group_dissimilar_samples_id(i, :)];  
    if ~isempty(find(i == train_group_id, 1))
        % if i is inside train_group_id, i.e. the current group belongs to train set   
        train_dissimilar_class = [train_dissimilar_class; [cur_group_similar(rand_id(1,:), :), cur_group_dissimilar(rand_id(2,:), :)] ]; 
    else
        test_dissimilar_class = [test_dissimilar_class; [cur_group_similar(rand_id(1,:), :), cur_group_dissimilar(rand_id(2,:), :)] ];         
    end
        
    disp('Intra-group samples done.');
    
end


% 3 - Get inter-group dissimilar pairs
% for train set
for i = 1 ; size(inter_train_group_pair, 2)
    % set id
    rand_id = [group_similar_samples_id(inter_train_group_pair(1, i), num_intra_group_similar*2+num_intra_group_dissimilar+1 : end); ...
               group_similar_samples_id(inter_train_group_pair(2, i), num_intra_group_similar*2+num_intra_group_dissimilar+1 : end)];
    if mod(num_train_groups, 2) ~= 0
        % odd number
        if (i == size(inter_train_group_pair, 2))
            % the last group pair; one group appear twice
            rand_id = [rest_train_id; ...
                      group_similar_samples_id(inter_train_group_pair(2, i), num_intra_group_similar*2+num_intra_group_dissimilar+1 : end)];
        end
    end

    % get data
    group_1_similar = h5read(new_file_name, [info.Groups(inter_train_group_pair(1, i)).Name, '/similarity_preserved_trajs']);
    group_2_similar = h5read(new_file_name, [info.Groups(inter_train_group_pair(2, i)).Name, '/similarity_preserved_trajs']);
    
    % get pair
    train_dissimilar_class = [train_dissimilar_class; [group_1_similar(rand_id(1,:), :), group_2_similar(rand_id(2,:), :)] ];
end
% for test set
% for train set
for i = 1 ; size(inter_test_group_pair, 2)
    % set id
    rand_id = [group_similar_samples_id(inter_test_group_pair(1, i), num_intra_group_similar*2+num_intra_group_dissimilar+1 : end); ...
               group_similar_samples_id(inter_test_group_pair(2, i), num_intra_group_similar*2+num_intra_group_dissimilar+1 : end)];
    if mod(num_test_groups, 2) ~= 0
        % odd number
        if (i == size(inter_test_group_pair, 2))
            % the last group pair; one group appear twice
            rand_id = [rest_test_id; ...
                      group_similar_samples_id(inter_test_group_pair(2, i), num_intra_group_similar*2+num_intra_group_dissimilar+1 : end)];
        end
    end

    % get data
    group_1_similar = h5read(new_file_name, [info.Groups(inter_test_group_pair(1, i)).Name, '/similarity_preserved_trajs']);
    group_2_similar = h5read(new_file_name, [info.Groups(inter_test_group_pair(2, i)).Name, '/similarity_preserved_trajs']);
    
    % get pair
    test_dissimilar_class = [test_dissimilar_class; [group_1_similar(rand_id(1,:), :), group_2_similar(rand_id(2,:), :)] ];
end


disp('Inter-group samples done.');


% store the training datasets
disp('Storing similar and dissimilar sample pairs in file...');
train_file_name = 'test_imi_data_YuMi_training_dataset.h5';
% h5create(train_file_name, '/similar', size(similar_class));
% h5create(train_file_name, '/dissimilar', size(dissimilar_class));
% h5write(train_file_name, '/similar', similar_class);
% h5write(train_file_name, '/dissimilar', dissimilar_class);
h5create(train_file_name, '/train_similar', size(train_similar_class));
h5create(train_file_name, '/train_dissimilar', size(train_dissimilar_class));
h5create(train_file_name, '/test_similar', size(test_similar_class));
h5create(train_file_name, '/test_dissimilar', size(test_dissimilar_class));
h5write(train_file_name, '/train_similar', train_similar_class);
h5write(train_file_name, '/train_dissimilar', train_dissimilar_class);
h5write(train_file_name, '/test_similar', test_similar_class);
h5write(train_file_name, '/test_dissimilar', test_dissimilar_class);


% further divide into x_train and x_test
%{
num_similar_samples = size(similar_class, 1);
num_dissimilar_samples = size(dissimilar_class, 1);
y_similar = ones(num_similar_samples, 1) .* 1; % labels
y_dissimilar = zeros(num_dissimilar_samples, 1) .* 0;
% 3/4 of samples for training (ratio is 1:3), btw, randomize the order
similar_train_id = sort(randperm(num_similar_samples, round(num_similar_samples * 3 / 4)));
similar_test_id = setdiff(randperm(num_similar_samples), similar_train_id);
dissimilar_train_id = sort(randperm(num_dissimilar_samples, round(num_dissimilar_samples * 3 / 4)));
dissimilar_test_id = setdiff(randperm(num_dissimilar_samples), dissimilar_train_id);
% construct training samples
x_train = [similar_class(similar_train_id, :); dissimilar_class(dissimilar_train_id, :)];
x_test = [similar_class(similar_test_id, :); dissimilar_class(dissimilar_test_id, :)];
y_train = [y_similar(similar_train_id, :); y_dissimilar(dissimilar_train_id, :)];
y_test = [y_similar(similar_test_id, :); y_dissimilar(dissimilar_test_id, :)];
%}
disp('Divide sample pairs into x_train and x_test..');
x_train = [train_similar_class; train_dissimilar_class];
y_train = [ones(size(train_similar_class, 1), 1); zeros(size(train_dissimilar_class, 1), 1)];
x_test = [test_similar_class; test_dissimilar_class];
y_test = [ones(size(test_similar_class, 1), 1); zeros(size(test_dissimilar_class, 1), 1)];
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

disp('All done.');
