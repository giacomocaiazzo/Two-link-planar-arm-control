%% System matrices

% State order: [theta1, theta1_dot, theta2, theta2_dot].
A = [0 1 0 0;32.79 0 6.625 0;0 0 0 1;-67.38 0 -8.92 0];
B = [0 0; 4.65 -7.94;0 0;-7.94 27.27];
C = [1 0 0 0;0 0 1 0];
D = zeros(2,2);

%% Controllability and observability analysis
Mr = ctrb(A,B);
Mc = obsv(A,C);
sys = ss(A,B,eye(4),zeros(4,2));
eig(sys);


%% LQR with prescribed decay rate and reference precompensation

alfa = 0;
Qaelements = [0.01 0.01 0.01 0.01];
Raelements = [1 1];
Qa = diag(Qaelements);
Ra = diag(Raelements);
[Ka,Sa,Pa]=lqr(A+alfa*eye(4),B,Qa,Ra);
sysCla = feedback(sys,Ka);
transferFunctionPosa = [sysCla(1,1) sysCla(1,2);sysCla(3,1) sysCla(3,2)];
gainMatrixa = dcgain(transferFunctionPosa^-1);

%% LQR with reference precompensation

Qelements = [1 1 1 1];
Relements = [1 1];
Q = diag(Qelements);
R = diag(Relements);
[K,S,P] = lqr(sys,Q,R);
sysCl = feedback(sys,K);
transferFunctionPos = [sysCl(1,1) sysCl(1,2);sysCl(3,1) sysCl(3,2)];
gainMatrix = dcgain(transferFunctionPos^-1);


%% LQR with integral action

Qielements = [1 1 1 1 1 1];
Rielements = [1 1];
Qi = diag(Qielements);
Ri = diag(Rielements);
Aaug = [A zeros(4,2);-C zeros(2,2)];
Baug = [B;zeros(2,2)];
Caug = [C zeros(2,2)];
Daug = D;

sysAug = ss(Aaug,Baug,Caug,Daug);
[Ki,Si,Pi] = lqr(sysAug,Qi,Ri);

%% Integral LQR weight tuning: trial 2
Qielements = [10 0 10 0 10 1];
Qi = diag(Qielements);
[Ki,Si,Pi] = lqr(sysAug,Qi,Ri);

%% Integral LQR weight tuning: trial 3
Rielements = [1 1];
Qielements = [300 0 100 0 300 100];
Qi = diag(Qielements);  
Ri = diag(Rielements);
[Ki,Si,Pi] = lqr(sysAug,Qi,Ri);

%% Integral LQR with prescribed decay rate

Qielements = 0.1*[1 1 1 1 1 1];
Rielements = [1 1];
Qi = diag(Qielements);
Ri = diag(Rielements);
alfa = 0.5;
[Ki,Si,Pi] = lqr(Aaug+alfa*eye(6),Baug,Qi,Ri);

opt = RespConfig;
opt.Amplitude=0.26;

%% Closed-loop interconnection with integral action
sys.InputName={'Torque1','Torque2'};
sys.OutputName={'P1','V1','P2','V2'};
sysItg = ss(zeros(2,2),eye(2),Ki(:,5:6),D);
sysItg.InputName={'errorP1','errorP2'};
sysItg.OutputName={'IntegralAction1','IntegralAction2'};
stateFeedbackMatrixfin = ss([],[],[],Ki(:,1:4));
stateFeedbackMatrixfin.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrixfin.OutputName = {'sfaction1','sfaction2'};
Sumint1 = sumblk('errorP1=reference1-P1');
Sumint2 = sumblk('errorP2=reference2-P2');
Sumi = append(Sumint1,Sumint2);
SumT1 = sumblk('Torque1=-IntegralAction1-sfaction1');
SumT2 = sumblk('Torque2=-IntegralAction2-sfaction2');
SumT = append(SumT1,SumT2);
sysCli = connect(sys,stateFeedbackMatrixfin,sysItg,Sumi,SumT,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfP1Cli = sysCli(1,1)+sysCli(1,2);
tfP2Cli = sysCli(3,1)+sysCli(3,2);
step(tfP1Cli,opt)
step(tfP2Cli,opt)


%% LQG: LQR and Kalman state estimation


%% Pole-placement observer

pObs2 = [-3000 -3001 -3002 -3003];
L2 = place(A',C',pObs2);
L2 = L2';
Aobs2=A-L2*C;
Bobs2=[B-L2*D L2];
Cobs2=eye(4);
Dobs2=zeros(4,4);

%% Kalman state estimator

Qtildeelements = 1e-12*[1 1 1 1];
Rtildeelements = [0.0043/5 0.0043/5];
Qtilde=diag(Qtildeelements); 
Rtilde=diag(Rtildeelements);
sys4Kal = ss(A,[B eye(4)],C,[zeros(2,6)]);
[kalmf,L,P] = kalman(sys4Kal,Qtilde,Rtilde);
AobsK=kalmf.A;
BobsK=kalmf.B;
CobsK=eye(4);
DobsK=zeros(2,4);

