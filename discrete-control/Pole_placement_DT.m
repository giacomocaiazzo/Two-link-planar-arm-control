%% System matrices

% State order: [theta1, theta1_dot, theta2, theta2_dot].
A = [0 1 0 0;32.79 0 6.625 0;0 0 0 1;-67.38 0 -8.92 0];
B = [0 0; 4.65 -7.94;0 0;-7.94 27.27];
C = [1 0 0 0;0 0 1 0];
D = zeros(2,2);

%% Zero-order-hold discretization
Ts = 0.1;
sys = ss(A,B,eye(4),zeros(4,2));
sysdis = c2d(sys,Ts,'ZOH');
[Ad,Bd,Cd,Dd]=ssdata(sysdis);

%% Reference and transient specifications
opt = RespConfig;
opt.Amplitude=0.26;
PO = 5;
Ta = 20;

%% Discrete-time pole placement
zita=abs(log(PO/100))/sqrt(pi^2+log(PO/100)^2);
omegan=4/(zita*Ta);

p1 = -omegan*zita + omegan*sqrt(1-zita^2)*1i;
p2 = -omegan*zita - omegan*sqrt(1-zita^2)*1i;
p3 = -10*abs(p1); 
p4 = p3-1;
p5 = p4-1;
p6 = p5-1;

pcc = [p1 p2 p3 p4];
zcc = exp(pcc*Ts);
K = place(Ad,Bd,zcc);

%% Closed-loop interconnection and reference precompensation
sysdis.InputName={'Torque1','Torque2'};
sysdis.OutputName={'P1','V1','P2','V2'};
stateFeedbackMatrix = ss([],[],[],K,Ts);
stateFeedbackMatrix.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrix.OutputName = {'sfaction1','sfaction2'};
SumT1 = sumblk('Torque1=reference1-sfaction1');
SumT2 = sumblk('Torque2=reference2-sfaction2');
SumT = append(SumT1,SumT2);
sysClk = connect(sysdis,stateFeedbackMatrix,SumT,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfMatrixPos = [sysClk(1,1) sysClk(1,2);sysClk(3,1) sysClk(3,2)];
gainMatrix = (dcgain(tfMatrixPos))^-1;
sysGM = ss([],[],[],gainMatrix);
sysGM.InputName = {'reference1','reference2'};
sysGM.OutputName = {'ref1_comp','ref2_comp'};
SumT1comp = sumblk('Torque1=ref1_comp-sfaction1');
SumT2comp = sumblk('Torque2=ref2_comp-sfaction2');
SumTcomp = append(SumT1comp,SumT2comp);
sysClkcomp = connect(sysdis,stateFeedbackMatrix,sysGM,SumTcomp,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfP1comp = sysClkcomp(1,1)+sysClkcomp(1,2);
tfP2comp = sysClkcomp(3,1)+sysClkcomp(3,2);
step(tfP1comp,opt);
step(tfP2comp,opt);


%% Integral augmentation and discretization
% Adaug = [Ad zeros(4,2);-Cd(1,:) zeros(1,2);-Cd(3,:) zeros(1,2)];
% Bdaug = [Bd;zeros(2,2)];
% Cdaug = [Cd(1,:) zeros(1,2);Cd(3,:) zeros(1,2)];
% Ddaug = Dd;

Aaug = [A zeros(4,2);-C zeros(2,2)];
Baug = [B;zeros(2,2)];
Caug = [C zeros(2,2)];
Daug = D;
sysAug = ss(Aaug,Baug,Caug,Daug);

sysdisAug = c2d(sysAug,Ts,'ZOH');
[Adaug,Bdaug,Cdaug,Ddaug]=ssdata(sysdisAug);

%% Pole placement for the augmented model
pcci = [p1 p2 p3 p4 p5 p6];
zcci = exp(pcci*Ts);
Kfinal = place(Adaug,Bdaug,zcci);

%% Closed-loop interconnection with integral action
sysItg = ss(zeros(2,2),eye(2),Kfinal(:,5:6),D);
sysItgdis = c2d(sysItg,Ts,'ZOH');
sysItgdis.InputName={'errorP1','errorP2'};
sysItgdis.OutputName={'IntegralAction1','IntegralAction2'};
stateFeedbackMatrixfin = ss([],[],[],Kfinal(:,1:4));
stateFeedbackMatrixfin.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrixfin.OutputName = {'sfaction1','sfaction2'};
Sumint1 = sumblk('errorP1=reference1-P1');
Sumint2 = sumblk('errorP2=reference2-P2');
Sumi = append(Sumint1,Sumint2);
SumT1 = sumblk('Torque1=-IntegralAction1-sfaction1');
SumT2 = sumblk('Torque2=-IntegralAction2-sfaction2');
SumT = append(SumT1,SumT2);
sysCli = connect(sysdis,stateFeedbackMatrixfin,sysItgdis,Sumi,SumT,{'reference1','reference2'},{'P1','V1','P2','V2'});
step(sysCli(1,1)+sysCli(1,2),opt);
step(sysCli(3,1)+sysCli(3,2),opt);



%% Discrete-time LQR
Q = diag([1 1 1 1]);
R = diag([1 1]);
[K] = dlqr(Ad,Bd,Q,R);


%% Discrete-time LQR with integral action
Qi = diag([10 10 10 10 10 10]);
Ri = diag([0.1 0.1]);
[Ki] = dlqr(Adaug,Bdaug,Qi,Ri);

%% Reference precompensation for discrete-time LQR
sysCldis = feedback(sysdis,K);
transferFunctionPos = [sysCldis(1,1) sysCldis(1,2);sysCldis(3,1) sysCldis(3,2)];
gainMatrix = dcgain(transferFunctionPos^-1);




