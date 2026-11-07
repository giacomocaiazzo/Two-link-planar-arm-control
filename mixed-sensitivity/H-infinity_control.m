%% System matrices

% State order: [theta1, theta1_dot, theta2, theta2_dot].
A = [0 1 0 0;32.79 0 6.625 0;0 0 0 1;-67.38 0 -8.92 0];
B = [0 0; 4.65 -7.94;0 0;-7.94 27.27];
C = [1 0 0 0;0 0 1 0];
D = zeros(2,2);

%% Controllability and observability analysis
Mr = ctrb(A,B);
Mc = obsv(A,C);
sys = ss(A,B,C,D);
eig(sys);




%% Mixed-sensitivity weights and controller synthesis



Ws=[zpk(makeweight(2000,[60,1],1/2000)) 0; 0 zpk(makeweight(2000,[30,1],1/2000))];
Wt=[zpk(makeweight(1/2000,[60,1],2000)) 0;0 zpk(makeweight(1/2000,[30,1],2000))]
Wk=diag([80,40])

[Kmix,CL,gamma]=mixsyn(sys,Ws,Wt,Wk);

%% Closed-loop sensitivity functions
L=sys*Kmix;
S=minreal(tf(eye(2))+L)^-1;
T=minreal(L*S);
sigma(T)

% Controller state-space matrices.
[Ak,Bk,Ck,Dk]=ssdata(Kmix);
    
%% Measurement and process noise covariances
Rt=diag([0.00086 0.00086]);
Qt=1e-12*eye(4);

