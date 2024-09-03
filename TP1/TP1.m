%%
clear all;close all;clc;
s=tf('s');

%Constantes
Qi=0.000133333;
d=10.65e-3;
l_inf=10e-2;
l_sup=40e-2;
L=0.9;
g=9.8;

%Nuevas Constantes
A=(pi/4)*d.^2;
alpha=(l_sup-l_inf)/L;
beta=l_inf;

%Chequeo del resultado de los cálculos teóricos
%Puntos de Equilibrio
h_eq=0.45;
u_eq=Qi/(sqrt(2*g*h_eq)*A);



G=-(sqrt(2*g*h_eq)*A)/(beta.^2+2*beta*alpha*h_eq+(alpha*h_eq).^2);
p=-(0.5*sqrt(2*g/h_eq)*u_eq*A)/(beta.^2+2*beta*alpha*h_eq+(alpha*h_eq).^2)-((Qi-sqrt(2*g*h_eq)*u_eq*A)*(2*beta*alpha+2*alpha.^2*h_eq))/(beta.^2+2*beta*alpha*h_eq+(alpha*h_eq).^2).^2;
P=G/(s-p);

h_eqs=[0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8];
Plantas={};
Labels={};
for i = 1:length(h_eqs)
    u_eqs=Qi/(sqrt(2*g*h_eqs(i))*A);
    G=-(sqrt(2*g*h_eqs(i))*A)/(beta.^2+2*beta*alpha*h_eqs(i)+(alpha*h_eqs(i)).^2);
    p_i=-(0.5*sqrt(2*g/h_eqs(i))*u_eqs*A)/(beta.^2+2*beta*alpha*h_eqs(i)+(alpha*h_eqs(i)).^2)-((Qi-sqrt(2*g*h_eqs(i))*u_eqs*A)*(2*beta*alpha+2*alpha.^2*h_eqs(i)))/(beta.^2+2*beta*alpha*h_eqs(i)+(alpha*h_eqs(i)).^2).^2;
    P_i=G/(s-p_i);
    Plantas{i}=P_i;
    Labels{i}=['h de eq:' num2str(h_eqs(i)) 'm'];
end
figure();
bode(Plantas{:})
grid on;
legend(Labels);
title('Respuesta en frecuencia para distintas condiciones iniciales');

%%
figure();
bode(P);
grid on;
legend('Planta');
title('Bode de la planta');

Kp=-4.536;
Ki=-0.03208;
C=Kp+Ki/s;

%C=zpk([p],[0],-1);
figure();
grid on;
bode(P*C);

figure();
L=P*C;
step(L/(1+L));






