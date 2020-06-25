import h5py

if __name__ == '__main__':

    ### Preparations
    file_name = './test_imi_data_YuMi.h5'


    ### For generating new trainging datasets
    '''
    del_dataset_list = ['resampled_normalized_flattened_oritraj']

    group_name_list = []
    with h5py.File(file_name, 'a') as f:
        # get the list of group names
        for key in f.keys():
            group_name_list.append(key)
        # iterate over each group
        for group_name in group_name_list:
            # check the kill list
            for del_dataset in del_dataset_list:
                if (del_dataset in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset]

    print('Deletion done.')
    '''

    ### For generating new f_seq and others
    '''
    # Preparations
    group_name = 'fengren_1'
    del_dataset_list = ['keypoint_list',
                        'pos_and_glove_id',
                        'quat_id',
                        'time_range',
                        'pass_time',
                        'pass_points',
                        'f_seq']

    with h5py.File(file_name, 'a') as f:
        # check the kill list
        for del_dataset in del_dataset_list:
            if (del_dataset in f[group_name].keys()):
                del f[group_name+'/'+del_dataset]

    print('Deletion done.')
    '''
    
    
    ### For clearing learned DMP weights and others
    # Preparations
    group_name = 'fengren_1'
    del_dataset_list_prefix = ['Mu',
                               'Sigma',
                               'Weights',
                               'Yr',
                               'sIn']
    del_dataset_list_suffix = ['_lrw',
                               '_lew',
                               '_rew',
                               '_rw']

    with h5py.File(file_name, 'a') as f:
        # check the kill list
        for del_dataset_prefix in del_dataset_list_prefix:
            for del_dataset_suffix in del_dataset_list_suffix:
                if (del_dataset_prefix+del_dataset_suffix in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset_prefix+del_dataset_suffix]

    print('Deletion done.')


    ### For clearing original starts and goals
    # Preparations
    group_name = 'fengren_1'
    del_dataset_list_prefix = ['lrw',
                               'lew',
                               'rew',
                               'rw']
    del_dataset_list_suffix = ['_goal',
                               '_start']

    with h5py.File(file_name, 'a') as f:
        # check the kill list
        for del_dataset_prefix in del_dataset_list_prefix:
            for del_dataset_suffix in del_dataset_list_suffix:
                if (del_dataset_prefix+del_dataset_suffix in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset_prefix+del_dataset_suffix]

    print('Deletion done.')    
    
    
    ### For clearing resampled quaternion and glove angle trajectories
    # Preparations
    group_name = 'fengren_1'
    del_dataset_list_prefix = ['l',
                               'r']
    del_dataset_list_suffix = ['_wrist_quat_resampled',
                               '_glove_angle_resampled']

    with h5py.File(file_name, 'a') as f:
        # check the kill list
        for del_dataset_prefix in del_dataset_list_prefix:
            for del_dataset_suffix in del_dataset_list_suffix:
                if (del_dataset_prefix+del_dataset_suffix in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset_prefix+del_dataset_suffix]

    print('Deletion done.')      
    
    
    
    
    
    
    
