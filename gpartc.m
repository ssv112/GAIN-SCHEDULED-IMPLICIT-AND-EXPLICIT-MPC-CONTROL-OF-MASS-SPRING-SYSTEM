clc;
clear;
%%Plant model is constructed
%Constants are edfined
Ktheta = 1280.2;  % Torsional rigidity
Kt = 10;          % Motor constant
Jm = 0.5;         % Motor inertia
Jl = 50*Jm;       % Load inertia
Rho = 20;         % Gear ratio
Betam = 0.1;      % Rotor viscous friction
Betal = 25;       % Load viscous friction
        % parameters
Delta_t =0.1;     % Sampling interval
Np =20;           % Prediction Horizon
Nc =8;            % Control Horizon
% rw1=0.001;        % controller weight 1/Tuning parameter 1
% rw2=0.01;         % controller weight 2/Tuning parameter 2
GR=20%Gear Ratio
%%Xp=states variables
%%Xp=[ThetaL OmegaL ThetaM OmegaM]'
%% State space matrix of the plant model is given as
A = [0 1 0 0;-Ktheta/Jl -Betal/Jl Ktheta/(Rho*Jl) 0;0 0 0 1;Ktheta/(Jm*Rho) 0 -Ktheta/(Jm*Rho^2) -(Betam+Kt^2/GR)/Jm];
B = [0; 0; 0; Kt/(GR*Jm)];
C = [1 0 0 0];
D = [0];

DC_motor = ss(A,B,C,D)
[Ad,Bd,Cd,Dd]=c2dm(A,B,C,D,Delta_t);

Ts=0.1;
desp=c2d(DC_motor,Ts,'ZOH')

Ap=desp.A;
Bp=desp.B;
Cp=desp.C;
Dp=desp.D;

Np=20;
Nc=8;

[Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Ap, Bp, Cp, Nc, Np)

rw=0.001;

a=0.7;

N=3;
Nsim=100;
u_min=-60;
u_max=60;


Q=C_e'*C_e;
R=rw*eye(1,1);

xm=[0;0.1;0;0.0];
[M0,Lzerot]=Mdu(a,N,1,Np);
[M1]=Mu(a,N,1,Np);
M=[M1;-M1];
Xf=[0;0;0;0;0];

y=0

u=0;
Nc=Np;

 r=ones(Nsim+1,1);
 XX=[];
 XF=[];
 [Omega,Psi]=dmpc(A_e,B_e,a,N,Np,Q,R);
for kk = 1:Nsim
up=u;
gamma=[(u_max-up)*ones(Nc,1);(-u_min+up)*ones(Nc,1)];
eta=QPhild(Omega,Psi*Xf,M,gamma);
deltau=Lzerot*eta;
u=u+deltau;
deltau1(kk)=deltau;
u1(kk)=u;
y1(kk)=y;

xm_old=xm;
xm=Ap*xm+Bp*u;
y=Cp*xm;

Xf=[xm-xm_old;(y-r(kk+1))];

end

subplot(311)
plot(y1,'linewidth',2.3)
xlabel('Sampling Instant')
ylabel ('output')
legend('system Output response for a=0.5')
subplot(312)
plot(u1,'linewidth',2.3)
xlabel('Sampling Instant')
ylabel('Control input')
legend( 'Control Input for a=0.5')
subplot(313)
 plot(deltau1,'linewidth',2.3)
 xlabel('Sampling Instant')
 ylabel('Control Increments ')
legend( 'control increments for a=0.5')
