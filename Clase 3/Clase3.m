
vector_filtrado = out.d3(905:1505)+4.4;
figure();

N=length(vector_filtrado);
t=linspace(0,N*0.01,N);

Y= vector_filtrado(3:N);
X=[vector_filtrado(2:(N-1)),vector_filtrado(1:(N-2))];

pseudo_inv = inv(transpose(X)*X);
w=-pseudo_inv*transpose(X)*Y
z=roots([1 w(1) w(2)])

T=0.01;
P_1 = log(z(1))/T;
P_2 = log(z(2))/T;



s = tf("s");
H = 1/((s-P_1)*(s-P_2))
figure();
pzplot(H);
[NUM,DEM]=tfdata(H,'v');
beta = DEM(3);
alpha = DEM(2);
A=[0 1; -beta -alpha]
B=[0; 1]
C=[1 0]
D=0;

sys = ss(A,B,C,D);
[salida,tiempo] = initial(sys,[15,0]);
figure();
hold on;
grid on;
plot(tiempo,salida)
plot(t,vector_filtrado);
