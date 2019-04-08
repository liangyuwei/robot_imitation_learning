function index_func = perform_gtw_on_traj(num_steps, traj_dataset)
%% This function performs GTW on the trajectories and returns the aligned new trajectories.

% num_steps is not unique, but should be greater than the length of any trajectory    
% 

prSet(1);
% tag = 3; l = 300; m = 3;
tag = 3; l = num_steps; m = 3;

%% parameter
inp = 'linear'; qp = 'matlab'; dp = 'c';
parGtw = st('nItMa', 50, 'th', 0);
parDtw = st('nItMa', 50, 'th', 0, 'inp', inp);
parPimw = st('nItMa', 50, 'th', 0, 'lA', 1, 'lB', 1);
parCca = st('d', 2, 'lams', 0);
parFtw = st('nItMa', 2, 'th', 0, 'lam', 0, 'nor', 'n', 'qp', qp, 'inp', inp);


%% use my data as src
aliT = []; % ground-truth, set to []
Xs = cell(length(traj_dataset), 1);
for i = 1 : length(traj_dataset)
    Xs{i} = traj_dataset{i, 1}'; % 10 x 1 cell
end

%% basis
% ns = cellDim(Xs, 2);
ns = cellDim(Xs, 2);
bas = baTems(l, ns, 'pol', [3 .4], 'tan', [3 .6 1]);

%% utw (initialization)
aliUtw = utw(Xs, bas, aliT);

%% pdtw
% aliPdtw = pdtw(Xs, aliUtw, aliT, parDtw);

%% pddtw
% aliPddtw = pddtw(Xs, aliUtw, aliT, parDtw);

%% pimw
% aliPimw = pimw(Xs, aliUtw, aliT, parPimw, parDtw);

%% gtw
aliGtw = gtw(Xs, bas, aliUtw, aliT, parGtw, parCca, parFtw);

%% Obtain aligned trajectories
% traj_dataset_aligned = cell(length(traj_dataset), 1);
% for i = 1 : length(traj_dataset)
%     traj_dataset_aligned{i} = traj_dataset{i}(round(aliGtw.P(:, i)), :);
% end
index_func = aliGtw.P;

%% show result
% shAliCmp(Xs, Xs, {aliGtw}, aliT, parCca, parDtw);%{aliPdtw, aliPddtw, aliPimw, aliGtw}, aliT, parCca, parDtw);

%% show basis
% shGtwPs(bas{1}.P);

