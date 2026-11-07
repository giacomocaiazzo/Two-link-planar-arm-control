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

%% Control specifications
PO = 5;
Ta = 15;

%% Pole placement
zita=abs(log(PO/100))/sqrt(pi^2+log(PO/100)^2);
omegan=4/(zita*Ta);

p1 = -omegan*zita + omegan*sqrt(1-zita^2)*1i;
p2 = -omegan*zita - omegan*sqrt(1-zita^2)*1i;
p3 = -10*abs(p1); 
p4 = p3-1;
p5 = p4-1;
p6 = p5-1;

pcc = [p1 p2 p3 p4]; 
K = place(A,B,pcc);

%% Closed-loop system interconnection
sys.InputName={'Torque1','Torque2'};
sys.OutputName={'P1','V1','P2','V2'};
stateFeedbackMatrix = ss([],[],[],K);
stateFeedbackMatrix.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrix.OutputName = {'sfaction1','sfaction2'};
SumT1 = sumblk('Torque1=reference1-sfaction1');
SumT2 = sumblk('Torque2=reference2-sfaction2');
SumT = append(SumT1,SumT2);
sysClk = connect(sys,stateFeedbackMatrix,SumT,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfP1 = sysClk(1,1)+sysClk(1,2);
tfP2 = sysClk(3,1)+sysClk(3,2);
opt = RespConfig;
opt.Amplitude=0.26;
step(tfP1,opt);


%% Response with nonzero initial conditions

 t=linspace(0,60,500);
 u=ones(length(t),2);
 x0 = [1.7;0;0.2;0]; 
 lsim(sysClk,u,t,x0)

%% Reference precompensation

tfMatrixPos = [sysClk(1,1) sysClk(1,2);sysClk(3,1) sysClk(3,2)];
gainMatrix = (dcgain(tfMatrixPos))^-1;
sysGM = ss([],[],[],gainMatrix);
sysGM.InputName = {'reference1','reference2'};
sysGM.OutputName = {'ref1_comp','ref2_comp'};
SumT1comp = sumblk('Torque1=ref1_comp-sfaction1');
SumT2comp = sumblk('Torque2=ref2_comp-sfaction2');
SumTcomp = append(SumT1comp,SumT2comp);
sysClkcomp = connect(sys,stateFeedbackMatrix,sysGM,SumTcomp,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfP1comp = sysClkcomp(1,1)+sysClkcomp(1,2);
tfP2comp = sysClkcomp(3,1)+sysClkcomp(3,2);
step(tfP1comp,opt);
step(tfP2comp,opt);


%% Integral action
Aaug = [A zeros(4,2);-C zeros(2,2)];
Baug = [B;zeros(2,2)];
Caug = [C zeros(2,2)];
Daug = D;

rank(ctrb(Aaug,Baug))

pcci = [p1 p2 p3 p4 p5 p6]; 
Kfinal = place(Aaug,Baug,pcci);

sysItg = ss(zeros(2,2),eye(2),Kfinal(:,5:6),D);
sysItg.InputName={'errorP1','errorP2'};
sysItg.OutputName={'IntegralAction1','IntegralAction2'};
stateFeedbackMatrixfin = ss([],[],[],Kfinal(:,1:4));
stateFeedbackMatrixfin.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrixfin.OutputName = {'sfaction1','sfaction2'};
Sumint1 = sumblk('errorP1=reference1-P1');
Sumint2 = sumblk('errorP2=reference2-P2');
Sumi =   append(Sumint1,Sumint2);
SumT1 = sumblk('Torque1=-IntegralAction1-sfaction1');
SumT2 = sumblk('Torque2=-IntegralAction2-sfaction2');
SumT = append(SumT1,SumT2);
sysCli = connect(sys,stateFeedbackMatrixfin,sysItg,Sumi,SumT,{'reference1','reference2'},{'P1','V1','P2','V2'});

tfP1int = sysCli(1,1)+sysCli(1,2);
tfP2int = sysCli(3,1)+sysCli(3,2);
step(tfP1int,opt);
step(tfP2int,opt);

%% ITAE pole placement

wn = 2;

%% Reference precompensation

pITAE = roots([1 2.1*wn 3.4*wn^2 2.7*wn^3 wn^4]);
KITAE = place(A,B,pITAE);
sysClITAE = feedback(sys,KITAE);
stateFeedbackMatrixITAE = ss([],[],[],KITAE);
stateFeedbackMatrixITAE.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrixITAE.OutputName = {'sfaction1','sfaction2'};
tfMatrixPosITAE = [sysClITAE(1,1) sysClITAE(1,2);sysClITAE(3,1) sysClITAE(3,2)];
gainMatrixITAE = (dcgain(tfMatrixPosITAE))^-1;
sysGMI = ss([],[],[],gainMatrixITAE);
sysGMI.InputName = {'reference1','reference2'};
sysGMI.OutputName = {'ref1_comp','ref2_comp'};
SumT1compI = sumblk('Torque1=ref1_comp-sfaction1');
SumT2compI = sumblk('Torque2=ref2_comp-sfaction2');
SumTcompI = append(SumT1compI,SumT2compI);
sysClITAE = connect(sys,stateFeedbackMatrixITAE,SumTcompI,sysGMI,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfP1ITAE = sysClITAE(1,1)+sysClITAE(1,2);
tfP2ITAE = sysClITAE(3,1)+sysClITAE(3,2);
step(tfP1ITAE,opt);
step(tfP2ITAE,opt);

%% Integral action

piITAE = roots([1 3.25*wn 6.6*wn^2 8.6*wn^3 7.45*wn^4 3.95*wn^5 wn^6]);
KiITAE = place(Aaug,Baug,piITAE);
sysItgI = ss(zeros(2,2),eye(2),KiITAE(:,5:6),D);
sysItgI.InputName={'errorP1','errorP2'};
sysItgI.OutputName={'IntegralAction1','IntegralAction2'};
stateFeedbackMatrixII = ss([],[],[],KiITAE(:,1:4));
stateFeedbackMatrixII.InputName = {'P1','V1','P2','V2'};
stateFeedbackMatrixII.OutputName = {'sfaction1','sfaction2'};
Sumint1I = sumblk('errorP1=reference1-P1');
Sumint2I = sumblk('errorP2=reference2-P2');
SumiI = append(Sumint1I,Sumint2I);
SumT1I = sumblk('Torque1=-IntegralAction1-sfaction1');
SumT2I = sumblk('Torque2=-IntegralAction2-sfaction2');
SumTI = append(SumT1I,SumT2I);
sysCliI = connect(sys,stateFeedbackMatrixII,sysItgI,SumiI,SumTI,{'reference1','reference2'},{'P1','V1','P2','V2'});
tfP1ITAEI = sysCliI(1,1)+sysCliI(1,2);
tfP2ITAEI = sysCliI(3,1)+sysCliI(3,2);
step(tfP1ITAEI,opt);
step(tfP2ITAEI,opt);

%% Luenberger observer
pobs1 = -10*abs(p1);
pobs2 = pobs1-10;
pobs3 = pobs2-11;
pobs4 = pobs3-12;

obspoles = [pobs1 pobs2 pobs3 pobs4];
L = place(A',C',obspoles);
Aobs = A-(L'*C);
Bobs = [B L'];
Cobs = eye(4);
Dobs = zeros(4);


