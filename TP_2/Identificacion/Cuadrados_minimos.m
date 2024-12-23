bias=1.5;

%csvwrite('d3_mediciones.csv',out.d3(3188:4100))

writematrix(out.d3(1799:3365)+bias,'d3_medicion.csv');

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
rr=zeros(N,1);
rr(1)=vector_filtrado(1);
rr(2)=vector_filtrado(2);
for i=3:N
    rr(i)=w(1)*rr(i-1)+w(2)*rr(i-2);
end

T=0.01;
P_1 = log(z(1))/T;
P_2 = log(z(2))/T;
%P_1 = -0.2076/2 + 7.3044i
%P_2 = -0.2076/2 - 7.3044i

s = tf("s");
H = zpk([],[P_1 P_2], 1);
% figure();
% pzplot(H);

[NUM,DEM]=tfdata(H,'v');
beta = DEM(3);
alpha = DEM(2);
A=[0 1; -beta -alpha];
B=[0; 1];
C=[1 0];
D=0;

sys = ss(A,B,C,D);
[salida,tiempo] = initial(sys,[vector_filtrado(1),0],t);
figure();
hold on;
grid on;
r=(17.57+bias)*exp(real(P_1).*t).*cos(imag(P_1).*t);
figure();
title('Planta del péndulo sobreamortiguada')
plot(tiempo,salida);
hold on;
plot(t,vector_filtrado);
grid on;
title('Planta del péndulo ajustada');
legend('Modelado', 'Real');
xlabel('Tiempo');  
ylabel('Theta(Grados)');  
