function m = proMP_get_mu_sigma_combine(nbStates, nbVar, nbSamples, nbData, traj_samples, time_range)
%% This function processes pos(1), angle(2) and quaternion(4-dim) data.
% input: nbStates - Number of basis functions
%        nbVar - Dimension of position data (here: x1,x2)
%        nbSamples - Number of demonstrations
%        nbData - Number of datapoints in a trajectory
%        traj_samples - All demonstration samples, should be of the size (DOF, nbData, nbSamples)
%        time_range - Canonical time, internal clock

if nbVar == 1
    % pos or angle data
    m = proMP_get_mu_sigma(nbStates, nbVar, nbSamples, nbData, traj_samples, time_range);
elseif nbVar == 4
    % quaternion data
    m = proMP_get_mu_sigma(nbStates, nbVar-1, nbSamples, nbData, traj_samples(2:end, :, :), time_range);
%     for i = 2:3
%         m_tmp = proMP_get_mu_sigma(nbStates, nbVar-1, nbSamples, nbData, traj_samples(i+1, :, :), time_range);
%         m(i).phi = m_tmp.phi;
%         m(i).Psi = m_tmp.Psi;
%         m(i).w = m_tmp.w;
%         m(i).Mu_w = m_tmp.Mu_w;
%         m(i).Sigma_w = m_tmp.Sigma_w;
%     end
else
    disp('proMP: Input error, trajectory data should be 1-dim or 4-dim!!!');
    return;
end

end



