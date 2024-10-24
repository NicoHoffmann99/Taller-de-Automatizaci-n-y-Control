import control as ct
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
from sklearn.linear_model import LinearRegression
import cmath as cm


pendulo=np.loadtxt("d3_mediciones.csv",delimiter=',')
pendulo=pendulo[11:]
def parametros_recta_de_regresion(X,Y):
    #X=np.column_stack((np.ones((len(X),1)),X))
    w=np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(X),X)),np.transpose(X)),Y)
    return w

N=len(pendulo)
t=np.linspace(0,N*0.01,N)

Y=pendulo[2:N]
X=np.column_stack((pendulo[0:N-2],pendulo[1:N-1]))
w=parametros_recta_de_regresion(X,Y)


z=np.roots([1, -w[1], -w[0]])

T=0.01

p1=cm.log(z[0])/T 
p2=cm.log(z[1])/T 

H=ct.zpk([],[p1,p2],1)
num, dem = ct.tfdata(H)
dem=np.array(dem)
dem=np.array(dem[0])


A=[[0 , 1],[-dem[0,2], -dem[0,1]]]
print(A)
B=[0,1]
C=[1,0]
D=[0]

sys=ct.ss(A,B,C,D)

tiempo, salida = ct.initial_response(sys,T=t,X0=[pendulo[0],0])

plt.plot(tiempo,salida)
plt.plot(t,pendulo)
plt.grid()
plt.show()
























































