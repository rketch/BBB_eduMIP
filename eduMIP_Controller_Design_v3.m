% . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
% .
% .  eduMIP BeagleBone Blue Controller Design 3.0
% . 
% .  Title: edupMIP_Controller_Design_v3
% .  Description:  Designs two controllers to balance an inverted pendulum
% .  Author:  Robert Ketchum                  
% .  Date Created: April 7, 2021
% .  Last Modified: April 11, 2021
% . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

close all  % Close all files
clear      % Clear all variables
clc        % Clear command line 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 1: Characterize the robot's equations of motion 
% and find the inner and outer loop plants: G1 and G2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Vb = 7.4;       % battery voltage [V]
wf = 1760;      % motor no load angular velocity [rad/sec]
st = 0.003;     % stall torque of motor sans gearbox [N m]
Gr = 35.5555;   % gearbox gear ratio [unitless]
Im = 3.6*10^-8; % motor armature moment of inertia [kg m^2]
rw = 0.034;     % wheel radius [r]
mw = 0.027;     % mass of each wheel [kg]
mb = 0.180;     % mass of robot body [kg]
L = 0.0477;     % length from center of mass to wheel axis [m]
Ib=0.000263;   % Robot body moment of inertia [kg m^2]
g=9.81;        % gravity [m/s^2]

k=st/wf;      % torque constant [N m/s]

Iw=2*((mw*rw^2)/2+Gr^2*Im); % wheel inertia [kg m^2]

%transfer functions to be used in G1
BA=tf([mb*rw*L -2*Gr^2*k 0],[Iw+(mb+mw)*rw^2 2*Gr^2*k 0]);
CA=tf([2*Gr*st],[Iw+(mb+mw)*rw^2 2*Gr^2*k 0]);
ED=tf([Ib+mb*L^2 2*Gr^2*k -mb*g*L],[mb*rw*L -2*Gr^2*k 0]);
FD=tf([-2*Gr*st],[mb*rw*L -2*Gr^2*k 0]);

G1=PolyDiv(CA-FD,BA-ED);
G1=minreal(G1);

pole1=pole(G1); % poles of G1
zero1=zero(G1); % zeros of G1

% transfer functions to be used in G2
EF = tf([Ib+mb*L^2 2*Gr^2*k -mb*g*L],[-2*Gr*st]);
BC = tf([mb*rw*L -2*Gr^2*k 0],[2*Gr*st]);
AC = tf([Iw+(mb+mw)*rw^2 2*Gr^2*k 0],[2*Gr*st]);
DF = tf([mb*rw*L -2*Gr^2*k 0],[-2*Gr*st]);

G2=PolyDiv(EF-BC,AC-DF);
G2=minreal(G2);

pole2=pole(G2); % poles of G2
zero2=zero(G2); % zeros of G2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 2: Design the inner loop controller: D1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DT1 = 200;           % sampling fz [Hz]
cross1 = (DT1)/20;   % crossover fz [Hz]
wc1 = cross1*2*pi;   % crossover fz [rad/sec]
alpha1 = 4;          % gives us that p1/z1=4

p1=wc1*sqrt(alpha1); % -1 * lead pole
z1=wc1/sqrt(alpha1); % -1 * lead zero
z2=20;               % -1 * lag zero
p2=0;                % -1 * lag pole
k1=-10.5;            % gain
D1=tf([1 (z1+z2) z1*z2],[1 (p2+p1) p1*p2]);
Dk1=D1*k1;           % Continuous Time Inner Loop Controller
G1D1=G1*D1*k1;       % Open Loop Transfer Function
T1=k1*G1*D1/(k1*G1*D1+1);
T1=minreal(T1);      % Closed Loop Transfer Function
T1_step_info = stepinfo(T1);

%Convert from continuous to Discrete Time using Tustin's Approximation with
%Prewarping
prewarp1=2*(1-cos(wc1/DT1))/(1/DT1*wc1*sin(wc1/DT1)); % Prewarp factor
dop1=c2dOptions('Method','tustin','PrewarpFrequency',prewarp1);
Dz1=c2d(Dk1,1/DT1,dop1); % Discrete Time Inner Loop Controller

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 3: Design the outer loop controller, D2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DT2=20;          % sampling fz for outer loop [Hz]
cross2=(DT2)/20; % crossover fz [Hz]
wc2=cross2*2*pi; % crossover fz [rad/sec]

p4=tf([1],[1 20]);  % lead pole
z3=tf([1 1],[1]);   % lag zero
p3=tf([1],[1 8]);   % lag pole
k2=7;               % gain
D2=p3*z3*p4*k2;     % Continuous Time Controller 2
G2D2=G2*D2;         % Open Loop Transfer Function
T2=G2*D2/(G2*D2+1); 
T2=minreal(T2);     % Closed Loop Transfer Function
T2_step_info = stepinfo(T2);

% Converting outer loop controller into Discrete Time
prewarp2=2*(1-cos(wc2/DT2))/(1/DT2*wc2*sin(wc2/DT2)); % prewarp factor
dop2=c2dOptions('Method','tustin','PrewarpFrequency',prewarp2);
Dz2=c2d(D2,1/DT2,dop2); % Outer loop discrete transfer function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 4: Full System Cascaded Transfer Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L2=G2*T1*D2;
T2f=L2/(1+L2);
T2f=minreal(T2f); % Full system transfer function
T2f_step_info = stepinfo(T2f);


%%%%%%%%%%%%%%%%%%
%PART 4: Plotting
%%%%%%%%%%%%%%%%%%

%Plots for Inner Loop
figure(1)
rlocus(G1);
axis([-50 20 -2 2])
title("Root Locus of G1")
grid on
saveas(1,'G1_RL.png')

figure(2)
bode(G1);
title("Bode of G1")
grid on
saveas(2,'G1_Bode.png')

figure(3)
rlocus(G1D1/-k1);
grid on
title("Root Locus of -1*G1D1")
axis([-150 20 -100 100])
saveas(3,"G1D1_RL.png")

figure(4)
bode(G1D1)
grid on
title("Bode of Open Loop G1D1")
saveas(4,"G1D1_Bode.png")

figure(5)
step(T1);
grid on
%axis([0 0.3 0 1.4])
title("Step Response of T1")
saveas(5,"T1_Step.png")

%Plots for Outer Loop
figure(6)
rlocus(G2);
grid on
title("Root Locus of G2")
axis([-50 20 -2 2])
saveas(6,"G2_RL.png")

figure(7)
bode(G2);
title("Bode of G2")
grid on
saveas(7,"G2_Bode.png")

figure(8)
rlocus(G2D2/k2);
grid on
title("Root Locus of -1*G2D2")
axis([-50 20 -2 2])
saveas(8,"G2D2_RL.png")

figure(9)
bode(G2D2)
grid on
title("Bode of G2*D2")
saveas(9,"G2D2_Bode.png")

figure(10)
step(T2);
title("Step Response of T2")
%axis([0 2.5 -1 2])
grid on
saveas(10,"T2_Step.png")

%Plot for Full System
figure(11)
step(T2f)
%axis([0 2.5 -1 2])
grid on
title("Step Response of Full System")
saveas(11,"T2f_Step.png")