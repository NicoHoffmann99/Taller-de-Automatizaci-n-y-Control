%clase observadores

%cambiar los polos por los nuevos.
polos=[-0.1324 + 7.3044i, -0.1324 - 7.3044i];
T = 0.01;

polos_c = 5 * polos; %con esto hago que sean mas rapidos.

%P_Z = exp(real(polos_c).*T); 
P_Z=[0.920 0.9270];
s = tf('s');

H = zpk([],[polos(2) polos(1)],1);

[NUM,DEN]=tfdata(H,'v');
[A,B,C,D] = tf2ss(NUM,DEN);
A_t = [A(2,2) A(2,1); A(1,2) A(1,1)];


A_dt = eye(size(A_t,1))+A_t.*T;

%implementaciones digitales matrices espacio de estados.
A_d = eye(size(A,1))+A.*T;
B_d = B .* T;


C_t = [C(2) C(1)];

L = place(A_d',C',P_Z)';
L_t = place(A_d',-C_t',P_Z)';
%muy rapido es ruidoso y muy lento tarda en converger pero es menos
%sensible al ruido

A_r  = [1.0 0.01;-0.5337 0.9974];
C_r = [1 0];

L_r = place(A_r',C_r',P_Z)';

p = real(polos)*100;

%Sumando Observador del Sesgo
A_t_b = [A(2,2) A(2,1), 0; A(1,2) A(1,1), 0; 0, 0, 0];
C_t_b = [1, 0, 0; 0, 1, 1];
A_d_b = eye(size(A_t_b,1))+A_t_b.*T;

P_zb = [exp(p.*T) 0.95];
P_Z_bias=[0.8199, 0.8199, 0.98];
L_b = place(A_d_b',C_t_b',P_zb)';
