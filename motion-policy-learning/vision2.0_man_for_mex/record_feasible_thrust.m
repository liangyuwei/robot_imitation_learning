function record_feasible_thrust
%% This function iterates through the physically feasible joint space to 
%% record the needed thrusts under certain constraints for comparison with the optimized result.

%% Environment setting as well as motion requirements
global traj_height traj_radius mu_k angular_acc avg_r
global uLINK expected_zmp
D = 100;
ToRad = pi/180;

%% Iteration through the joint space
% Set up physical ranges of the joints
lb=[-48; -130; -100; -48; -130; -100];
ub=[115; 150; 60; 115; 150; 60];
the_test = [90, 0, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, 0, 0];

% variable for storage
the_stg = [];
pos_left_ankle_stg = [];
f_stg = [];
theta_stg = [];
dist_r_G_stg = [];

% create a .mat file for storage
savefile = 'record_feasible_thrust.mat';
save(savefile, 'the_stg', 'pos_left_ankle_stg', 'f_stg', 'theta_stg', 'dist_r_G_stg');

% Start iteration
for the3 = lb(1):0.5:ub(1) % the left leg
    for the4 = lb(2):0.5:ub(2)
        the5 = - (the3 + the4); % Constraint of left foot being horizontal to the base
        if the5 < lb(3) || the5 > ub(3)
            continue; % skip this iteration
        end
        
        for the9 = lb(4):0.5:ub(4) % the right leg
            for the10 = lb(5):0.5:ub(5)
                the11 = - (the9 + the10); % Constraint of the base being horizontal to the right foor, i.e. the ground
                
                
                if the11 < lb(6) || the11 > ub(6) % out of range
                    continue;
                end
                
                % set the joint configuration
                the_test = [90, 0, the3, the4, the5, 0, 0, -90, 0, the9, the10, the11, 0, 0];

                % display the message               
%                 ['Checking the = [', num2str(the_test), ']...']
                
                % Constraint of the left ankle's position
                R_Ly = -220 + 220 * sin(the11 * ToRad) - 140 * sin(the9 * ToRad) - 140 * sin(the3 * ToRad) + 220 * sin(the5 * ToRad);
                R_Lz = 40 + 220 * cos(the11 * ToRad) + 140 * cos(the9 * ToRad) - 140 * cos(the3 * ToRad) - 220 * cos(the5 * ToRad);
                if abs(R_Ly - (- traj_radius)) > 0.1 || abs(R_Lz - traj_height) > 0.1 % uLINK(7).p' ~= [0, expected_zmp(2) - traj_radius, traj_height] % norm(uLINK(7).p' - [0, expected_zmp(2) - traj_radius, traj_height]) <= eps
                    continue;
                end
                
                % display message for finding a configuration that satisfies the basic constraints 
                for ii = 1:10
                    ['Found one feasible: the_test = [', num2str(the_test), '].']
                end
                
                % update the robot's state for the calculation of theta and f   
                write(the_test);
                
                % Calculation of the theta and thrust f
                p_L = [uLINK(7).p(1), uLINK(7).p(2), uLINK(7).p(3)];
                p_zmp = expected_zmp;
                r_L = p_L - p_zmp;
                r_Ly = r_L(2);
                
                % CoM of the whole robot without the part of the swing leg's foot
                mc = 0; m_G =0; m_l = 0;
                for i = 1:15
                    if i ~= 6 && i ~= 7 && i~= 8 % eliminating the left foot's weight
                        mc = mc + uLINK(i).c * uLINK(i).m;
                        m_G = m_G + uLINK(i).m;
                    else
                        m_l = m_l + uLINK(i).m;
                    end
                end
                p_G = mc./m_G;
                r_G = p_G' - p_zmp;
                r_Gy = r_G(2);
                
                g = 9.8; % gravitational acceleration
                
                % required theta and thrust
                ankle_theta = atan(...
                    ( mu_k * avg_r * g * (m_G*r_Ly-m_G*D-m_l*D-m_G*r_Gy) - angular_acc * (-r_Ly + D) * (m_G * r_Gy ^2 + m_l * r_Ly ^2) ) ...
                    /((m_l*r_Ly+m_G*r_Gy) * (- r_Ly + D) * g) ...
                    );
                
                f = (m_G * r_Gy + m_l * r_Ly) * g / ((r_Ly - D) * cos(ankle_theta));
                
                % record data
                the_stg = [the_stg; the_test];
                pos_left_ankle_stg = [pos_left_ankle_stg; uLINK(7).p'];
                f_stg = [f_stg; f];
                theta_stg = [theta_stg; ankle_theta];
                dist_r_G_stg = [dist_r_G_stg; abs(r_G(2))];
                
                % save data regularly
                if size(f_stg) == 5
                    % create tmp variable
                    the_stg_tmp = the_stg;
                    pos_left_ankle_stg_tmp = pos_left_ankle_stg;
                    f_stg_tmp = f_stg;
                    theta_stg_tmp = theta_stg;
                    dist_r_G_stg_tmp = dist_r_G_stg;

                    % load recorded data
                    load(savefile);
                    
                    % append new data and save it
                    the_stg = [the_stg; the_stg_tmp];
                    pos_left_ankle_stg = [pos_left_ankle_stg; pos_left_ankle_stg_tmp];
                    f_stg = [f_stg; f_stg_tmp];
                    theta_stg = [theta_stg; theta_stg_tmp];
                    dist_r_G_stg = [dist_r_G_stg; dist_r_G_stg_tmp]
                    save(savefile, 'the_stg', 'pos_left_ankle_stg', 'f_stg', 'theta_stg', 'dist_r_G_stg');
                    
                    % clear varible and free space
                    clear the_stg_tmp pos_left_ankle_stg_tmp f_stg_tmp theta_stg_tmp dist_r_G_stg_tmp
                    the_stg = [];
                    pos_left_ankle_stg = [];
                    f_stg = [];
                    theta_stg = [];
                    dist_r_G_stg = [];
                end

            end
        end
        
        % display message regularly
%         ['Checked the = [', num2str(the_test), ']...']

    end
    
    % display message regularly
    ['Checked the3 = [', num2str(the3), ']...']
    
end

%% Save the rest of the data
% create tmp variable
the_stg_tmp = the_stg;
pos_left_ankle_stg_tmp = pos_left_ankle_stg;
f_stg_tmp = f_stg;
theta_stg_tmp = theta_stg;
dist_r_G_stg_tmp = dist_r_G_stg;

% load recorded data
load(savefile);

% append new data and save it
the_stg = [the_stg; the_stg_tmp];
pos_left_ankle_stg = [pos_left_ankle_stg; pos_left_ankle_stg_tmp];
f_stg = [f_stg; f_stg_tmp];
theta_stg = [theta_stg; theta_stg_tmp];
dist_r_G_stg = [dist_r_G_stg; dist_r_G_stg_tmp];
save(savefile, 'the_stg', 'pos_left_ankle_stg', 'f_stg', 'theta_stg', 'dist_r_G_stg');

% clear varibles and free space
clear the_stg_tmp pos_left_ankle_stg_tmp f_stg_tmp theta_stg_tmp dist_r_G_stg
the_stg = [];
pos_left_ankle_stg = [];
f_stg = [];
theta_stg = [];
dist_r_G_stg = [];
%% Record the environment settings
save(savefile, 'mu_k', 'angular_acc', 'traj_height', 'traj_radius', '-append');

end


