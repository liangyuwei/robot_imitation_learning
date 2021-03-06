import h5py

if __name__ == '__main__':

    ### Preparations
    file_name = './test_imi_data_YuMi.h5'
    group_name = 'kaoqin_2'#'fengren_1'

    print('Deleting data of group '+group_name+'...')

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
    
    
    ### For clearing robot setting and adjusted movements(for debug)
    
    # Preparations
    del_dataset_list_prefix = ['l_',
                               'r_']
    del_dataset_list_suffix = ['wrist',
                               'elbow',
                               'shoulder']

    with h5py.File(file_name, 'a') as f:
        # robot setting
        if ('robot_hand_length' in f[group_name].keys()):
          del f[group_name+'/robot_hand_length']
        # check the kill list of adjusted movements
        for del_dataset_prefix in del_dataset_list_prefix:
            for del_dataset_suffix in del_dataset_list_suffix:
                if (del_dataset_prefix+del_dataset_suffix+'_pos_adjusted' in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset_prefix+del_dataset_suffix+'_pos_adjusted']

    print('Deletion done.')      
    
    
    ### For clearing interpolated optimized joint trajectories
    
    # Preparations
    del_dataset_list_prefix = ['arm_traj_1']
    del_dataset_list_suffix = ['_linear',
                               '_spline',
                               '_pchip']

    with h5py.File(file_name, 'a') as f:
        # check the kill list of adjusted movements
        for del_dataset_prefix in del_dataset_list_prefix:
            for del_dataset_suffix in del_dataset_list_suffix:
                if (del_dataset_prefix+del_dataset_suffix in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset_prefix+del_dataset_suffix]

    print('Deletion done.') 
    


    ### For clearing robot setting and adjusted movements(for debug)
    
    # Preparations
    del_dataset_list_prefix = ['l_wrist_pos',
                               'r_wrist_pos',
                               'l_elbow_pos',
                               'r_elbow_pos']
    del_dataset_list_suffix = ['_nullspace']

    with h5py.File(file_name, 'a') as f:
        # check the kill list of adjusted movements
        for del_dataset_prefix in del_dataset_list_prefix:
            for del_dataset_suffix in del_dataset_list_suffix:
                if (del_dataset_prefix+del_dataset_suffix in f[group_name].keys()):
                    del f[group_name+'/'+del_dataset_prefix+del_dataset_suffix]

    print('Deletion done.')      
    
