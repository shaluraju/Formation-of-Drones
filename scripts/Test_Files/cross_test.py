#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 14:35:24 2021

@author: Shalu
"""

from TelloServer import SWARM, Telloserver
import numpy as np
from Shape_vectors import LineFormation, Character

from PID import PID
import matplotlib.pyplot as plt

#import timeit

"""
Tello Drones are connected to wifi dongles thus, their interface
id's are defined here so that, the control commands are sent to
correct drone.  
"""

wifi_interfaces = ["wls1",
                   "wlx9cefd5fb6d70",
                   "wlx9cefd5fb71cc",
                    "wlx9cefd5fb6f40",
                   "wlx9cefd5fb774c",
                   ]

droneslist = ['Tello1',
              'Tello4',
              'Tello5',
              'Tello6',
              'Tello8',
             ]


swarm = SWARM(wifi_interfaces, droneslist)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()

# =======================================================
# Initialize the PID controllers

drone1PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#---------------------------------------------------------------
drone2PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone2PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#--------------------------------------------------------------
drone3PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone3PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone3PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#---------------------------------------------------------------
drone4PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone4PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone4PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#---------------------------------------------------------------

drone5PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone5PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone5PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#--------------------------------------------------------------


for drone in allDrones:   
    drone.connect()  

for drone in allDrones:
    drone.takeoff()

rosClock.sleepForRate(5000)

i = 0
j = 1
Ts = 0.02 # sample time
theta = 1


# ------------- Cross Formation ------------------


try:
    while j < 1500: 
        
        pos_curr = []
        for drone in GroundTruth:   
            pos = drone.getPose()[0]
            #print("pos of drone is: ",pos)
            pos_curr.append(pos)

        print("pos of drone is: ",pos_curr)
        
        #print("Controlling 1")
        PIDvx1 = drone1PIDx.update(pos_curr[0][0], 0.0)
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], 0.0)
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], 1.0)
    
    
        goal = Character.form_cross(pos_curr)
        
        print("goal: ", goal)
    
        #print("Controlling 2")
        PIDvx2 = drone2PIDx.update(pos_curr[1][0], goal[0][0])
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], goal[0][1])
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], goal[0][2])
        
        PIDvx3 = drone3PIDx.update(pos_curr[2][0], goal[1][0])
        PIDvy3 = drone3PIDy.update(pos_curr[2][1], goal[1][1]) 
        PIDvz3 = drone3PIDz.update(pos_curr[2][2], goal[1][2])

        PIDvx4 = drone4PIDx.update(pos_curr[3][0], goal[2][0])
        PIDvy4 = drone4PIDy.update(pos_curr[3][1], goal[2][1]) 
        PIDvz4 = drone4PIDz.update(pos_curr[3][2], goal[2][2])
        
        PIDvx5 = drone5PIDx.update(pos_curr[4][0], goal[3][0])
        PIDvy5 = drone5PIDy.update(pos_curr[4][1], goal[3][1]) 
        PIDvz5 = drone5PIDz.update(pos_curr[4][2], goal[3][2])
        
#-----------------------------------------------------------------
        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        
        if j > 10:
            print("commanding 2 & 3")
            allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
            allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
            allDrones[3].cmdVelocity(PIDvx4, PIDvy4, PIDvz4, 0)
            allDrones[4].cmdVelocity(PIDvx5, PIDvy5, PIDvz5, 0)
        
        
        #print("PID 1: ",PIDvx1, PIDvy1, PIDvz1)
        #print("PID 2: ",PIDvx2, PIDvy2, PIDvz2)
        j+=1
        print("Iteration: ", j)
#        #print("Position : ", pos_curr)
        rosClock.sleepForRate(1/Ts)
        print("_____________________________________________________")
#    
#    #rosClock.sleepForRate(1000)

#    
#--------------------------------- Landing -------------------------------
    for drone in allDrones:
        drone.land()
    
    
except KeyboardInterrupt:
    
    print('emergency interruption!; aborting all flights ...')   
    
    Telloserver.Landing(GroundTruth, allDrones)
#    for i in range(2):
#        for drone in allDrones:
#            drone.emergency()
#        rosClock.sleepForRate(10)      
#    pass

else:

    for drone in allDrones:
        drone.land()

   
for drone in allDrones:   
     drone.Disconnect()

#plt.savefig('/home/sas-lab/catkin_ws/src/vicon_bridge/scripts/line_rotation_plots/1_try.png') 