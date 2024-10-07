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
% fig_bode_plantas = figure();
% bode(Plantas{:})
% grid on;
%legend(Labels);
%title('Respuesta en frecuencia para distintas condiciones iniciales');
%saveas(fig_bode_plantas,'bode_plantas_h_dif.png','png');

%%
% fig_planta = figure();
% bode(P);
% grid on;
% legend('Planta');
% title('Bode de la planta');
%saveas(fig_planta,'bode_planta.png','png');

%Kp=-4.536;
%Ki=-0.03208;
%C=Kp+Ki/s;

C=zpk([p],[0],-4);

%     fig_pzk = figure();
%     hold on;
%     pzplot(C*P);
%     pzplot(P);
%     legend('planta control', 'planta');

%saveas(fig_pzk,'pzk_planta_control.png','png');
% 
% fig_planta_control = figure();
% bode(P*C);
% grid on;
% %saveas(fig_planta_control,'bode_planta_control.png','png');
% 
% fig_rta_esc = figure();
% L=P*C;
% step(L/(1+L));
% grid on;
%saveas(fig_rta_esc,'rta_escalon.png','png');


a = 1; b = 0.00237; k = -4;
Ts=1;
C = k * (s + b) / s;
C_d=c2d(C,Ts,'zoh');
z = tf('z',Ts);

Cd_f = (-4*z+3.99052)/(z-1);
% Forward Euler
cd_fe = tf([k, -k+k*b*Ts],[1,-1],Ts);

% Backward Euler
cd_be = tf([k*(a+b*Ts),-k],[1,-1],Ts);

% Tustin
cd_t = c2d(C,Ts,'tustin');







