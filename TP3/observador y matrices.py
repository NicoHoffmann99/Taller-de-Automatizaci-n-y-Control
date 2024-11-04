
import numpy as np

from scipy import signal as sg

from scipy import signal
import cmath as cm
#Parametros
Q = 1/2
w = 15.61
h0 = 1



#coeficientes del numerador y denominador


num = [-170.57,0,0]

polos=[p1, p2, p_1, p_2]

#Ganancia
k = (w**2)*0.7
Ts=0.01
#Planta total
P=ct.zpk([0, 0], polos, -k)

num, den = ct.tfdata(P)
num = [-170.57047,0,0]
den = [1.00000000e+00, 3.14849512e+01, 3.05287026e+02, 1.72993434e+03, 1.29982373e+04]
A, B, C, D = signal.tf2ss(num, den)
print(A)

