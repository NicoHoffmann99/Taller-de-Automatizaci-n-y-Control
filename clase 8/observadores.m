%clase observadores


polos=[-1.0896 + 8.2827i, -1.0896 - 8.2827i];
T = 0.01;

polos_c = 5 * polos;

P_Z = exp(polos_c.*T); 

s = tf('s');

H = zpk([],[polos(1) polos(2)],1);

[NUM,DEN]=tfdata(H,'v');
[A,B,C,D] = tf2ss(NUM,DEN);

%implementaciones digitales matrices espacio de estados.
A_d = eye(size(A,1))+A.*T;
B_d = B .* T;



L = place(A_d',C',P_Z)';

%muy rapido es ruidoso y muy lento tarda en converger pero es menos
%sensible al ruido