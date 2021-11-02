%% PID Control Design

% Constants & Design Parameters

h3 = 10; % distance between the center of gravity of the rocket & the gimbaled Merlin 1D engine in consideration (meters) 
T = 845.22 * 10^3; % thrust of a Falcon 9 FT stage 1 Merlin 1D engine (Newtons)
J = 37576837; % moment of inertia of the Falcon 9 about the vertical axis (assumption: cylindrical body)
C = (h3*T)/J; % constant

s = tf('s');

% Plant TF, 'Gp'
Gp = zpk(minreal(C/s^2))

% These values were obtained by auto-tuning the PID controller in Simulink
P = 0.825493884458449;
I = 0.0468147782797737;
D = 5.24517904929436;
N = 5.45397025421619;
Gc = zpk(minreal(pid(P,I,D,N),1e-05))

L = zpk(minreal((Gc*Gp),1e-05))
T = zpk(minreal((L/(1 + L)),1e-05))
S = zpk(minreal((1 - T),1e-05))
Y = zpk(minreal((T*Gp),1e-05))
GpS = zpk(minreal((Gp*S),1e-05))

% Internal stability check
Y_stability = isstable(Y)
T_stability = isstable(T)
S_stability = isstable(S)
GpS_stability = isstable(GpS)

M2 = 1/getPeakGain(S) % M2-margin
BW = bandwidth(T) % bandwidth of the closed-loop
AE = getPeakGain(Y) % maximum actuator effort

figure(1)
bodemag(Y, S, T);
legend('Y','S','T');
print -depsc PID.eps;
