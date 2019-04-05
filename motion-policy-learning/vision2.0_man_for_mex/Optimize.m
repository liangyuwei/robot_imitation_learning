function q = Optimize(H)
%% This function optimizes the joint variables under the constraints of ...

% Record height for the computation of nonlinear constraints
global Hei
Hei = H; % what height???

ToRad = pi/180;

% Initial values
x0=[0;
    0;
    0;
    0;
    0;
    0].*ToRad;

% Linear Inequalities Constraints
A=[];
b=[]; % Set to [] if no inequalities exist

% Linear Equality constraints
Aeq=[1 1 -1 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0 ;
     0 0 0 1 1 -1 ;
     0 0 0 0 0 0 ;
     0 0 0 0 0 0]; % Only 6 joint variables are needed for this algorithm
                   % For the base to remain horizontal to the ground.
                   % q1+q2-q3 = 0 --> Is it due to different rotation
                   % directions?
beq=[0 0 0 0 0 0]';

% Variables' boundries(due to motor's physical limitation)
lb=[-100;
    -150;
    -48;
    -48;
    -150;
    -100
    ].*ToRad;
ub=[100;
    130;
    115;
    115;
    130;
    100].*ToRad;

% Execute optimization, to minimize @fun, under nonlinear constraint @mycon
[x, ~] = fmincon(@fun,x0,A,b,Aeq,beq,lb,ub,@mycon); 

% Return results
q = [x(1),x(2),x(3),x(4),x(5),x(6)] / ToRad;

