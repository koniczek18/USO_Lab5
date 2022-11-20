from scipy.integrate import odeint
import numpy as np
import scipy as sci
import matplotlib.pyplot as plt
import control

#rady
#opcja 1 - użyć control
#opcja 2 - zdefiniować model RAZEM z PID def model_pid(z,t) z<- x, całka z e
#tutaj a) zdefiniować macierze systemu
#      b) dx=A@+B@u
#      c) u=kp*e+kd*de+ki*calka(ei)
#      d) yd-y
#      e) y=C@x

#format z=[x`1,x`2,x`3,total_e,e`)]
def model_PID(z, t, A, B, C, yd, kp, ki, kd):
    x=np.array([[z[0]],[z[1]],[z[2]]]) #extract state variables
    total_e=z[3]                 #get error till now
    e=z[4]                       #get previous error
    y=C@x                        #calculate current response
    ep=yd-y[0]                      #calculate current error (PASS)
    u=kp*e+ki*total_e+kd*ep      #calculate signal
    total_e += e                 # change total error
    dx=A@x+B*u                   #calculate new state variables to pass to return (PASS)
    xp1=dx[0,0]               #reformat
    xp2 = dx[1,0]
    xp3 = dx[2,0]
    re=np.array([xp1,xp2,xp3,total_e,ep])
    return re

def zadanie1(active):
    if active:
        R1=2
        R2=5
        C1=0.5
        L1=2
        L2=0.5
        A=np.array([[-R2/L2,0,-1/L2],[0,-R1/L1,1/L1],[1/C1,-1/C1,0]])
        B=np.array([[0],[1/L1],[0]])
        C=np.array([1,0,0])
        D=np.array([0])
        t=np.linspace(0,5,101)
        response=odeint(model_PID, y0=[0, 0, 0, 0, 0], t=t, args=(A, B, C, 3, 1, 1, 1))
        plt.figure('System')
        plt.plot(t,response[:,0],label='x1')
        plt.plot(t, response[:, 1],label='x2')
        plt.plot(t, response[:, 2],label='x3')
        plt.plot(t, response[:, 3],label='total e')
        plt.plot(t, response[:, 4],label='e',linestyle='dotted')
        plt.legend()
        plt.show()

if __name__ == '__main__':
    zadanie1(True)