%% Youla Control Design

s = tf('s');

% Constants & Design Parameters

h3 = 10; % distance between the center of gravity of the rocket & the gimbaled Merlin 1D engine in consideration (meters) 
T = 845.22 * 10^3; % thrust of a Falcon 9 FT stage 1 Merlin 1D engine (Newtons)
J = 37576837; % moment of inertia of the Falcon 9 about the vertical axis (assumption: cylindrical body)
C = (h3*T)/J; % constant
Wn = 0.85; % natural frequency of the control system
K = (Wn^2)/C; % controller gain
Z = 2^-0.5; % damping ratio
tp = 1/(10*Wn); % time constant of the added pole 
tz = (20*2^(1/2))/17 + 2/17; % time constant of the added pole (to satisfy the 2nd interpolation condition (shown later))

% Plant TF, 'Gp'
Gp = zpk(minreal(C/s^2))

% Chosen Youla Parameter, 'Y' -> Y(0) = 0
Y = zpk(minreal(((K*s^2)*(tz*s + 1)/((s^2 + 2*Z*Wn*s + Wn^2)*(tp*s + 1))),1e-05))

% Complementary Sensitivity TF, 'T' -> T(0) = 1 (1st interpolation
% condition)
T = zpk(minreal((Y*Gp),1e-05))

% Sensitivity TF, 'S'
S = zpk(minreal((1-T),1e-05))

% Controller TF, 'Gc'
Gc = zpk(minreal((Y/S),1e-05))

% Return Ratio, 'L'
L = zpk(minreal((Gc*Gp),1e-05))

GpS = zpk(minreal((Gp*S),1e-05))

% Internal stability check
Y_stability = isstable(Y)
T_stability = isstable(T)
S_stability = isstable(S)
GpS_stability = isstable(GpS)

M2 = 1/getPeakGain(S) % M2-margin
BW = bandwidth(T) % Bandwidth of the closed-loop
AE = getPeakGain(Y) % Maximum actuator effort

figure(1)
bodemag(Y, S, T);
legend('Y','S','T');

%% Run this section first to calculate 'tz' to ensure that the second interpolation condition is satisfied

% d^k(T)/ds^k|(s=0) = 0, where k = 1 (since there is a double unstable pole
% (multiplicity ap = 2) in the plant at s = 0; k = ap - 1) -> 2nd
% interpolation condition

h3 = 10; % distance between the center of gravity of the rocket & the gimbaled Merlin 1D engine in consideration (meters) 
T = 845.22 * 10^3; % thrust of a Falcon 9 FT stage 1 Merlin 1D engine (Newtons)
J = 37576837; % moment of inertia of the Falcon 9 about the vertical axis (assumption: cylindrical body)
C = (h3*T)/J; % constant
Wn = 0.85; % natural frequency of the control system
K = Wn^2/C; % controller gain
Z = 2^-0.5; % damping ratio
tp = 1/(10*Wn); % time constant of the added pole

syms s tz

TF = ((K*C)*(tz*s + 1))/((s^2 + 2*Z*Wn*s + Wn^2)*(tp*s + 1))
dTF = diff(TF,s)
eqn = subs(dTF,s,0) == 0;
tz = solve(eqn,tz)
