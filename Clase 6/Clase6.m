%%
clear all;close all;clc;
s=tf('s');

%Planta
P=(-0.099863*s)/((s+10.14)*(s-10.14));

%Espacio de estados
num = [-0.099863 0];
den = [1 0 -102.8196];
[A, B, C, D] = tf2ss(num, den);

%Condición inical
phi_i=5*(pi);

%Muestreo
Ts=0.01;
Ps=(1-Ts/4)/(1+Ts/4);

%Control PID
kp=4200/2
ki=41530/2
kd=5/2
%C_pid=-(kp+ki/s+kd*s);
C_pid=-(kp+ki/s+kd*s)*(1/(s/2000+1));
%C_pid= -(kp + ki/s + (kd*s)/(s/1000+1));
L=C_pid * P;
S = minreal(L/(1+L));
figure();
bode(S);
grid on;




