path0 = cd;

cd 'lib/cell';
mex cellss.cpp;
mex oness.cpp;
mex zeross.cpp;
cd(path0);

cd 'src/ali/dtw';
mex dtwFord.cpp;
mex dtwBack.cpp;
mex dtwFordAsy.cpp;
cd(path0);

cd 'src/ali/help';
mex rowBd.cpp;
cd(path0);

cd 'src/ali/imw';
mex timewarp.cpp;
cd(path0);
