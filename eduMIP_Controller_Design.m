clear all
close all
clc

% . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
% .
% .  MAE 144 Final Project: eduMIP Balancing Robot Matlab Code
% .
% .
% .  Author:  Robert Ketchum                  
% .  Revised: 1/28/2018
% .
% .  Please note: This code uses functions to sort out transfer function
% .  polynomials. These functions may be found on page S-1 and S-2 here:
% .  http://numerical-renaissance.com/NR.pdf
% .  All credit for these functions goes to Prof. Bewley at UCSD. I would
% .  like to thank Prof. Bewley and Jeffrey Friesen for their guidance on
% .  this project.
% . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 1: Characterizing the robot's equations of motion and finding G1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vb=7.4; %battery voltage
wf=1760; %wheel free run speed
st=0.003; %stall torque of motor sans gearbox
Gr=35.5555; %gearbox gear ratio
Im=3.6*10^-8; %motor armature
rw=0.034; %wheel radius
mw=0.027; %mass of each wheel
mb=0.180; %mass of robot body
L=0.0477; %length from center of mass to wheel axis
Ib=0.000263; %Robot body inertia
g=9.81; %gravity

k=st/wf; %torque constant

Iw=2*((mw*rw^2)/2+Gr^2*Im); %wheel inertia

%transfer functions to be used in G1, G2
BA=tf([mb*rw*L -2*Gr^2*k 0],[Iw+(mb+mw)*rw^2 2*Gr^2*k 0]);
CA=tf([2*Gr*st],[Iw+(mb+mw)*rw^2 2*Gr^2*k 0]);
ED=tf([Ib+mb*L^2 2*Gr^2*k -mb*g*L],[mb*rw*L -2*Gr^2*k 0]);
FD=tf([-2*Gr*st],[mb*rw*L -2*Gr^2*k 0]);

num=CA-FD;
den=BA-ED;
G1=PolyDiv(num,den);
G1=minreal(G1);

pole1=pole(G1);
zero1=zero(G1);

%%%%%%%%%%%%%%%%%%%%%
%PART 2: Controller 1
%%%%%%%%%%%%%%%%%%%%%

DT1=200; %sampling fz
cross1=(DT1)/25; %crossover fz in hz
wc1=60; %crossover fz in rad/sec
alpha1=4; %gives us that p1/z1=4

p1=wc1*sqrt(alpha1); %-1* lead pole
z1=wc1/sqrt(alpha1); %-1* lead zero
z2=20; %-1* lag zero
p2=0; %-1* lag pole
k1=-10*0.3; %gain
poleD=[-p2 -p1];
zeroD=[-z2 -z1];
D1=tf([1 (z1+z2) z1*z2],[1 (p2+p1) p1*p2]);
Dk1=D1*k1; %Continuous Time Controller 1
G1D1=G1*D1*k1;
T1=k1*G1*D1/(k1*G1*D1+1);
T1=minreal(T1); %Closed Loop Transfer Function

%Convert from continuous to Discrete Time using Tustin's Approximation with
%Prewarping
prewarp1=2*(1-cos(wc1/DT1))/(1/DT1*wc1*sin(wc1/DT1)); %prewarp factor
dop1=c2dOptions('Method','tustin','PrewarpFrequency',prewarp1);
Dz1=c2d(Dk1,1/DT1,dop1); %Discrete Time Controller 1
[numcellz,dencellz]=tfdata(Dz1);
numz=cell2mat(numcellz);
denz=cell2mat(dencellz);
syms tk
discretenum=numz(1)+numz(2)/tk+numz(3)/tk^2;
discreteden=denz(1)+denz(2)/tk+denz(3)/tk^2;
inputz=iztrans(discretenum);
outputz=iztrans(discreteden);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 3: G2 and Controller 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

G2=tf([-(Ib+mb*L^2+mb*rw*L) 0 mb*L*g],[Iw+(mb+mw)*rw^2+mb*rw*L 0 0]);
G2=minreal(G2);

pole2=pole(G2);
zero2=zero(G2);
DT2=20; %sampling fz for outer loop
cross2=(DT2)/20; %crossover fz in hz
wc2=cross2*2*pi; %crossover fz in rad/sec
alpha2=5; %gives us that p4/z4=5 in lead pole

p4=tf([1],[1 wc2*sqrt(alpha2)]); %lead pole
z3=tf([1 1],[1]); %lag zero
p3=tf([1],[1 5]); %lag pole
k2=0.3; %gain
D2=p3*z3*p4*k2; %Continuous Time Controller 2
G2D2=G2*D2; %Open Loop Transfer Function
T2=G2*D2/(G2*D2+1); %Closed Loop Transfer Function

%%%Converting Controller 2 into Discrete Time
prewarp2=2*(1-cos(wc2/DT2))/(1/DT2*wc2*sin(wc2/DT2)); %prewarp factor
dop2=c2dOptions('Method','tustin','PrewarpFrequency',prewarp2);
Dz2=c2d(D2,1/DT2,dop2); %Discrete Time Controller 2
[numcell2,dencell2]=tfdata(Dz2);
numz2=cell2mat(numcell2);
den2=cell2mat(dencell2);
syms tk
discretenum2=numz2(1)+numz2(2)/tk+numz2(3)/tk^2;
discreteden2=den2(1)+den2(2)/tk+den2(3)/tk^2;
input2=iztrans(discretenum2); %input of system
output2=iztrans(discreteden2); %output of system

%Cascaded transfer functions
L2=G2*T1*D2;
T2f=L2/(1+L2);


%%%%%%%%%%%%%%
%PART 4: Plots
%%%%%%%%%%%%%%

%Plots for Inner Loop
figure(1)
subplot(2,1,1)
axis([-50 20 -2 2])
rlocus(G1);
title("Root Locus of G1")
grid on
hold on
subplot(2,1,2)
rlocus(G1D1/-k1);
grid on
title("Root Locus of G1*D1")
axis([-100 20 -100 100])

figure(2)
bode(G1);
title("Bode of G1")
grid on

figure(3)
step(T1);
grid on
axis([0 2 0 3])
title("Step Response of Inner Loop")

figure(4)
bode(G1D1)
grid on
title("Bode of G1*D1")

figure(5)
bode(D1)
title("Bode of D1")
grid on

%Plots for Outer Loop
figure(6)
subplot(2, 1, 1)
rlocus(G2);
grid on
title("Root Locus of G2")
axis([-50 20 -2 2])
hold on
subplot(2, 1, 2)
rlocus(G2D2/k2);
grid on
title("Root Locus of G2*D2")
axis([-50 20 -2 2])

figure(7)
bode(G2);
title("Bode of G2")
grid on

figure(8)
bode(D2)
grid on
title("Bode of D2")

figure(9)
bode(G2D2)
grid on
title("Bode of G2*D2")

figure(10)
step(T2);
title("Step Response of T2")
axis([0 20 -1 2])
grid on


%Plot for Full System
figure(11)
step(T2f)
axis([0 20 -1 2])
grid on
title("Step Response of Both Loops")

%saveas(1,"Inner_Loop_RL.png")
%saveas(3,"Inner_Loop_Step.png")
%saveas(4,"Inner_Loop_Bode.png")
%saveas(6,"Outer_Loop_RL.png")
%saveas(9,"Outer_Loop_Bode.png")
%saveas(11,"Outer_Loop_Step.png")
