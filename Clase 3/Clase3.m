bias=1.5;
%8500----10200
%csvwrite('d3_mediciones.csv',out.d3(3188:4100))

%funcion que imprime la simulacion obtenida/pasada y la estimacion con
%cuadrados minimos. Devuelve la funcion de transferencia de la estimación
%obtenida por cuadrados minimos.

function [NUM,DEN] = plots_estimaciones(muestras,alta)
 N = length(muestras);
 time = linspace(0,N*0.01,N);
 
 %cuadrados minimos
 Y = muestras(3:N); X = [muestras(2:(N-1)),muestras(1:(N-2))];
 
 %Pseudoinversa
 W = pinv(X)*Y;
 %raices
 Z = roots([1 -W(1) -W(2)]);
 
 %Polos complejos
 T=0.01; %escala tiempo muestreo
 P1 = log(Z(1))/T
 P2 = log(Z(2))/T
 P1=real(P1)/2+1i*imag(P1)
P2=real(P2)/2+1i*imag(P2)

 %Transferencia
 H = zpk([],[P1 P2],1);

 %saco data y armo sistema.
 [NUM,DEN]=tfdata(H_total,'v');
 beta = DEN(3); alpha = DEN(2);
 A=[0 1; -beta -alpha]; B=[0; 1];
 C=[1 0]; D=0;

 sys = ss(A,B,C,D);
 if(alta == 1)
      [salida,tiempo] = initial(sys,[muestras(1),0],time);
 else

    opt = stepDataOptions('StepAmplitude', 30);
    [salida,tiempo] = step(sys,opt);
 end
 figure();
 hold on;
 grid on;
 plot(tiempo,salida);
 plot(time,muestras);

end 


%quiero hacer el ploteo de los datos
%viendo la simulacion de slx las muestras deben ser de 8500 a 10200, para
%el pendulo arranco desde un pico mas limpio (2do pico positivo) --> 8598
%el del brazo agarro solo un par de puntos cuando esta moviendose


writematrix(out.d3(8598:10200) - out.d3(10121),'m_pend.csv');
writematrix( out.d2(8500:8600) - out.d2(8500),'m_brazo.csv');
writematrix(out.d1(8500:8600) - out.d1(8500),'m_ref.csv');
data_p = readtable('m_pend.csv');
data_b = readtable('m_brazo.csv');
data_r = readtable('m_ref.csv');

m_pend = data_p.Var1;
m_brazo = data_b.Var1; 
m_ref = data_r.Var1;



[num_p,den_p] = plots_estimaciones(m_pend,0);
[num_b,den_b] = plots_estimaciones(m_brazo,1);
[num_r,den_r] = plots_estimaciones(m_ref,1);







writematrix(out.d2(8500:8600),'d3_medicion.csv');

data = readtable('d3_medicion.csv');
vector_filtrado = data.Var1;
%vector_filtrado=out.d3(3188:4100)+bias; %-out.d3(1755);
N=length(vector_filtrado);
t=linspace(0,N*0.01,N);

Y= vector_filtrado(3:N);
X=[vector_filtrado(2:(N-1)),vector_filtrado(1:(N-2))];

%pseudo_inv = pinv(X);
w=pinv(X)*Y;
z=roots([1 -w(1) -w(2)]);

T=0.01;
P_1 = log(z(1))/T
P_2 = log(z(2))/T
P_1=real(P_1)/2+1i*imag(P_1)
P_2=real(P_2)/2+1i*imag(P_2)
s = tf("s");
H = zpk([],[P_1 P_2], 1);
% figure();
% pzplot(H);

[NUM,DEN]=tfdata(H,'v');
beta = DEN(3);
alpha = DEN(2);
A=[0 1; -beta -alpha];
B=[0; 1];
C=[1 0];
D=0;

sys = ss(A,B,C,D);
[salida,tiempo] = initial(sys,[vector_filtrado(1),0],t);
figure();
hold on;
grid on;
r=(vector_filtrado(1)+bias)*exp(real(P_1/2).*t).*cos(imag(P_1).*t);
plot(tiempo,salida);
plot(t,vector_filtrado);
