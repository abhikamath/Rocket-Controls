%% H-infinity Optimization-based Control Design

% Courtesy of Professor Francis Assadian & TA Kevin Mallon

s=tf('s');
 
Gp = 0.224931119135972/s^2;

sysg=ss(Gp);
[Ag,Bg,Cg,Dg]=ssdata(sysg);

Ag=Ag-0.08*eye(2); % slightly shift A to avoid poles on jw axis

[num,den]=ss2tf(Ag,Bg,Cg,Dg);
Gpn=tf(num,den); % perturbed plant

%Hinf shaping filter
Wp = (db2mag(80)*((1/1.8177)*s + 1)^2)/((100)*s + 1)^2;
Wu = 0.01675;
Wd = s/3.469;
  
%Hinf Controller Computation
ssga_=augtf(Gpn,Wp,Wu,Wd);
[sys3,sscl]=hinfsyn(ssga_); 
  
% Hinf Controller
Gc=zpk(minreal(tf(sys3)))

% Results
L=zpk(minreal((Gc*Gp),1e-05))
Y=zpk(minreal((Gc/(1+Gc*Gp)),1e-05))
T=zpk(minreal((Y*Gp),1e-05))
S=zpk(minreal((1-T),1e-05))
GpS=zpk(minreal((Gp*S),1e-05))

% Internal stability check
Y_stability = isstable(Y)
T_stability = isstable(T)
S_stability = isstable(S)
GpS_stability = isstable(GpS)

M2 = 1/getPeakGain(S) % M2-margin
BW = bandwidth(T) % Bandwidth of the closed-loop
AE = getPeakGain(Y) % Maximum actuator effort

NP = getPeakGain(Wp*S) % Nominal Performance
RS = getPeakGain(Wd*T) % Robust Stability
RP = getPeakGain((Wp*S) + (Wd*T)) % Robust Performance

figure(1)
bodemag(Y,S,T)
legend('Y','S','T','location','southeast');

figure(2)
bodemag(T,S,1/Wp)
legend('T','S','1/Wp','location','southeast')

figure(3)
bodemag(Wp,S)
title('Frequency Response')
legend('Wp','S','location','southeast')

figure(4)
bodemag(T,S,1/Wd)
legend('T','S','1/Wd','location','southeast')
