%{
marco.villafuerte.j@uni.pe
%}

clear, clc, close all;
t = 0:0.01:2;
s = tf('s');           % s for Laplace operations

% Transfer Function and Feedback Loop
num = [1 100];
den = [1 8 65];
sysLA = tf(num,den);
sysLC = feedback(sysLA,1);

% No-Controlled system outputs
figure(1)             
step(sysLC)
title("No-controlled Unit-Step response")
legend('Output')

figure(2)                 
lsim(sysLC,t,t)
title("No-controlled Ramp-Step response")
legend('Output')

%{
Transfer function of the PID Controller:
Gc =  K(as+1)(bs+1)
     --------------
          s
%}

% Design requirements:
Mfdeseado = 60;       % Desired margin phase(°)
ess       = 0.1;      % steady state error (to a ramp input)

% Original error constant Kv, to a ramp input
Kv = 1/ess;

% K Gain of the controlled system
K= Kv/dcgain(sysLA);   


%%%%%%%%%%%%%%%%%%% I - Controller  %%%%%%%%%%%%%%%%%%%
%{
Transfer function of the I Controller:
Gc =     K
     -------
         s
%}

%  Margin and cross frequency of the I Controlled system, Gc = K/s:
[MgActual, MfActual,Wcgain,Wcphase] = margin(K/s*sysLA);

% Bode diagram of the I Controlled system, Gc = K/s:
figure(3);
margin(K/s*sysLA);
txt = {'Bode Diagram of the I - Controlled system'; 
       ['Pm = ' num2str(MfActual) '° (at ' num2str(Wcphase) ' rads/s)'] };
title(txt);

% First ZERO of the controller (as+1) will add 60°
a = tan(deg2rad(60))/Wcphase;


%%%%%%%%%%%%%%%%%%% PI - Controller  %%%%%%%%%%%%%%%%%%%
%{
Transfer function of the PI Controller:
Gc =  K(as+1)
     ----------
         s
%}

%  Margin and cross frequency of the PI Controlled system, Gc = K(as+1)/s:
[MgActual, MfActual,Wcgain,Wcphase] = margin(K*(a*s+1)/s*sysLA);

% Bode diagram of the PI Controlled system, Gc = K(as+1)/s:
figure(4)
margin(K*(a*s+1)/s*sysLA);
txt = {'Bode Diagram of the PI - Controlled system'; 
       ['Pm = ' num2str(MfActual) '° (at ' num2str(Wcphase) ' rads/s)'] };
title(txt);

% Second ZERO of the controller (bs+1) will add 30°
b = tan(deg2rad(30))/Wcphase;
%* Wcphase is now 12.6, because it was re-defined in the last script.


%%%%%%%%%%%%%%% Full PID - Controller  %%%%%%%%%%%%%%%%%
%{
Transfer function of the PID Controller:
Gc =  K(as+1)(bs+1)
     ---------------
         s
%}

%  Margin and cross frequency of the PID Controlled system, Gc = K(as+1)(bs+1)/s:
[MgActual, MfActual,Wcgain,Wcphase] = margin(K*(a*s+1)*(b*s+1)/s*sysLA);

% Bode diagram of the PID Controlled system, Gc = K(as+1)(bs+1)/s:
figure(5)
margin(K*(a*s+1)*(b*s+1)/s*sysLA);
txt = {'Bode Diagram of the full PID - Controlled system'; 
       ['Pm = ' num2str(MfActual) '° (at ' num2str(Wcphase) ' rads/s)'] };
title(txt);


%%%%%%%%% OUPUTS - Full PID Controlled system  %%%%%%%%%%%
% Controller Transfer Function
Gc = K*(a*s+1)*(b*s+1)/s;

% Feedback of the full PID Controlled System
sysLCControlado = feedback(sysLA*Gc,1);

% Controlled system outputs
figure(6)             
step(sysLCControlado)
title("Controlled Unit-Step response")
legend('Output')

figure(7)             
lsim(sysLCControlado,t,t)
title("Controlled Ramp-Step response")
legend('Output')



