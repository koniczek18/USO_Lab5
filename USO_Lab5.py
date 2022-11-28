from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt


#format z=[x`1,x`2,x`3,e)]
def model_PID(z, t, A, B, C, yd, kp, ki, kd):
    x=np.array([[z[0]],[z[1]],[z[2]]]) #extract state variables
    total_e=z[3]                 #get error till now
    ep=-C@A@x                 #get previous error
    y=C@x                        #calculate current response
    e=yd-y[0]                      #calculate current error (PASS)
    u=kp*e+ki*total_e+kd*ep      #calculate signal
    dx=A@x+B*u                   #calculate new state variables to pass to return (PASS)
    xp1=dx[0,0]               #reformat
    xp2 = dx[1,0]
    xp3 = dx[2,0]
    re=np.array([xp1,xp2,xp3,e,u[0]])
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
        response=odeint(model_PID, y0=[0, 0, 0, 0,0], t=t, args=(A, B, C, 3, 76, 0, 0))
        plt.figure('System with PID')
        plt.plot(t,response[:,0],label='x1 = y')
        plt.plot(t, response[:, 1],label='x2')
        plt.plot(t, response[:, 2],label='x3')
        plt.plot(t, response[:, 3],label='Total Error',linestyle='dotted')
        plt.legend()
        plt.show()

def zadanie2_3(active1,active2):
    if active1:
        R1 = 2
        R2 = 5
        C1 = 0.5
        L1 = 2
        L2 = 0.5
        A = np.array([[-R2 / L2, 0, -1 / L2], [0, -R1 / L1, 1 / L1], [1 / C1, -1 / C1, 0]])
        B = np.array([[0], [1 / L1], [0]])
        C = np.array([1, 0, 0])
        D = np.array([0])
        t = np.linspace(0, 5, 101)
        ku = 75.5
        Tu = 5 / 3
        kp=0.6*ku
        ki=1.2*ku/Tu
        kd=0.075*ku*Tu
        value=3*np.ones_like(t)
        response = odeint(model_PID, y0=[0, 0, 0, 0,0], t=t, args=(A, B, C, 3, kp,ki,kd))
        plt.figure('System with PID - Ziegler-Nichols')
        plt.plot(t, response[:, 0], label='x1 = y')
        plt.plot(t, response[:, 1], label='x2')
        plt.plot(t, response[:, 2], label='x3')
        plt.plot(t, response[:, 3], label='Total Error', linestyle='dotted')
        plt.plot(t,value,label='Ud',linestyle='dashed',color='r')
        plt.legend()
        plt.grid()
        plt.show()
        if active2:
            ISE=np.sum(response[:,3]*response[:,3])
            print(ISE)
            ITSE=np.sum(response[:,3]*response[:,3]*t)
            print(ITSE)
            IAE=np.sum(np.abs(response[:,3]))
            print(IAE)
            ITAE = np.sum(t*np.abs(response[:, 3]))
            print(ITAE)
            OPT=np.sum(response[:,3]*response[:,3]+response[:,4]*response[:,4])
            print(OPT)
        """
        Ziegler-Nichols
        ISE=35.32767989912245
        ITSE=53.310997786375346
        IAE=50.002857142274685
        ITAE=104.4431630154637
        OPT=643398.8753429432
        """

def testowanie(active):
    if active:
        R1 = 2
        R2 = 5
        C1 = 0.5
        L1 = 2
        L2 = 0.5
        A = np.array([[-R2 / L2, 0, -1 / L2], [0, -R1 / L1, 1 / L1], [1 / C1, -1 / C1, 0]])
        B = np.array([[0], [1 / L1], [0]])
        C = np.array([1, 0, 0])
        D = np.array([0])
        t = np.linspace(0, 5, 101)
        value = 3 * np.ones_like(t)
        ku = 75.5
        Tu = 5 / 3
        ise_p = 114
        ise_I = 68
        ise_d = 8.3
        response = odeint(model_PID, y0=[0, 0, 0, 0, 0], t=t, args=(A, B, C, 3, ise_p,ise_I,ise_d))
        plt.figure('Testowanie')
        plt.plot(t, response[:, 0], label='x1 = y')
        plt.plot(t, response[:, 1], label='x2')
        plt.plot(t, response[:, 2], label='x3')
        plt.plot(t, response[:, 3], label='Total Error', linestyle='dotted')
        plt.plot(t, value, label='Ud', linestyle='dashed', color='r')
        plt.legend()
        plt.grid()
        #plt.show()
        ISE = np.sum(response[:, 3] * response[:, 3])
        print(ISE)
        ITSE = np.sum(response[:, 3] * response[:, 3] * t)
        #print(ITSE)
        IAE = np.sum(np.abs(response[:, 3]))
        #print(IAE)
        ITAE = np.sum(t * np.abs(response[:, 3]))
        #print(ITAE)
        OPT = np.sum(response[:, 3] * response[:, 3] + response[:, 4] * response[:, 4])
        #print(OPT)

if __name__ == '__main__':
    zadanie1(False)
    zadanie2_3(False,False)
    testowanie(True)