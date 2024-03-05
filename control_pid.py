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

elapsedTime = 0
prevTime = time.time()

while elapsedTime <= S:
    
    t = time.time()
    elapsedTime = t - prevTime
    
    #  Error Calculation
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
    
    v = np.dot(K,e)
    v = np.clip(v, -0.2, 0.2)
    u = np.dot(T,v)/0.05
    
    #Integrator (Simulated Pose)
    R = np.array([[np.cos(p[2]), -np.sin(p[2]), 0],
                  [np.sin(p[2]),  np.cos(p[2]), 0],
                  [        0,          0, 1]])
    L = np.array([[1, -1, -(L+l)],
                  [1,  1,  (L+l)],
                  [1,  1, -(L+l)],
                  [1, -1,  (L+l)]])
    L = np.linalg.pinv(L)
    pp = np.dot(np.dot(R,L),u)*0.05
    p = p + pp*elapsedTime
    
    print("Pose: "+str(p))
    print("Vel:  "+str(pp))
    prevTime = t