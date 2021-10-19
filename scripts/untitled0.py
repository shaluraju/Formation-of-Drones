# -*- coding: utf-8 -*-
"""
Created on Mon Oct 11 22:03:04 2021

@author: sssha
"""

from TelloServer import SWARM, Telloserver
import numpy as np
from Shape_vectors import LineFormation
# import time
from PID import PID

wifi_interfaces = ["wlx9cefd5fb6d84",
                   "wlx9cefd5fb774c"
                    #"wlx9cefd5faea28",
                   #"wlx9cefd5fae83e"
                   ]

droneslist = ['Tello4',
              'Tello1'
              #'Tello3'
             ]


swarm = SWARM(wifi_interfaces, droneslist)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()

# =======================================================
# Initialize the PID controllers

drone1PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

for drone in allDrones:   
    drone.connect()  
    drone.takeoff()


rosClock.sleepForRate(3000)

i = 0

while i < 50: 
    
    pos_curr = []
    for drone in GroundTruth:   
        pos = drone.getPose()[0]
        print("pos of drone is: ",pos)
        pos_curr.append(pos)
    
    PIDvx1 = drone1PIDx.update(pos_curr[0][0], 0)
    PIDvy1 = drone1PIDy.update(pos_curr[0][1], -0.5)
    PIDvz1 = drone1PIDz.update(pos_curr[0][2], 0.8)
    i += 1
    rosClock.sleepForRate(1000)

itr = 0
max_itr = 2000
itr_sw = 300
Ts = 0.02 # sample time
theta = 0

try:

    while theta < 64:    
        
        pos_curr = []
        for drone in GroundTruth:   
            pos = drone.getPose()[0]
            print("pos of drone is: ",pos)
            pos_curr.append(pos)
            
        goal = LineFormation.d_rotation(pos_curr, 0.5, theta)

        PIDvx1 = drone1PIDx.update(pos_curr[0][0], goal[0])
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], goal[1])
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], goal[2])
        
        
        print('PIDv1:', PIDvx1, PIDvy1, PIDvz1)
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)


        rosClock.sleepForRate(1/Ts)
        theta += 1

except KeyboardInterrupt:
    
    print('emergency interruption!; aborting all flights ...')   
    for i in range(2):
        for drone in allDrones:
            drone.emergency()
        rosClock.sleepForRate(10)      
    pass

else:

    for drone in allDrones:
        drone.land()

   
for drone in allDrones:   
     drone.Disconnect()










