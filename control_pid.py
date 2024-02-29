import numpy as np
import time

# Inicialización de variables
p = np.array([0, 0, 0])
pp = np.array([0, 0, 0])


K = 0.1*np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
e = np.array([0, 0, 0])
ei = np.array([0, 0, 0])
eprev = np.array([0, 0, 0])
eprima = np.array([0, 0, 0])
prevTime = 0

# Bucle principal
S = 60
t = 0

pd = np.array([2, 2, 0])

initial = time.time()
while t <= S:
    t = time.time()-initial
    #  Error Calculation
    elapsedTime = t - prevTime
    prevTime = t

    e = pd - p
    #ei += eprima
    #ed = (e - eprev)
    #eprev = e

    # Parámetros
    pi = np.pi
    alpha = p[2] + pi/4
    L = 0.425/2
    l = 0.44/2

    T = np.array([[np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha), -(L+l)],
                  [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha),  (L+l)],
                  [np.sqrt(2)*np.cos(alpha),  np.sqrt(2)*np.sin(alpha), -(L+l)],
                  [np.sqrt(2)*np.sin(alpha), -np.sqrt(2)*np.cos(alpha),  (L+l)]])
    
    pp = np.dot(K,e)
    wheelVel = np.dot(T,pp)/0.05
    print(pp)

    #Integrator (Simulated Pose)
    p = p + pp*t

    elapsedTime = t - prevTime
    prevTime = t