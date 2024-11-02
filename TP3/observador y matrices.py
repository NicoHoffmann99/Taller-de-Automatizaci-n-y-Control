import control as ct
import numpy as np
import scipy as sc
import cmath as cm
#Parametros
Q = 1/2
w = 15.61
h0 = 1


#Polos brazos
p1 =- w/(2*Q) + w*np.sqrt((1/(2*Q))**2-1)
p2 = - w/(Q*2) - w*np.sqrt((1/(2*Q))**2-1)
#Polos pendulo
p_1 = -0.13247559170053874+1j*7.302437935150943
p_2 = -0.13247559170053874-1j*7.302437935150943

polos=[p1, p2, p_1, p_2]

#Ganancia
k = (w**2)*0.7
Ts=0.01
#Planta total
P=ct.zpk([0, 0], polos, -k)

num, den = ct.tfdata(P)
print(np.array(num[0]))
print(den)
A, B, C, D = ct.tf2ss(num, den)
print(A)