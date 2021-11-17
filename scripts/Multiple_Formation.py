#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 15 17:45:36 2021

@author: shalu
"""

from TelloServer import SWARM, Telloserver
import numpy as np
from Shape_vectors import *

from PID import PID
import matplotlib.pyplot as plt

#import timeit

wifi_interfaces = ["wls1",
                   "wlx9cefd5fb6d70",
                   "wlx9cefd5fb6f40",
                    "wlx9cefd5fb71cc",
                   #"wlx9cefd5fb774c",
                   ]

droneslist = ['Tello6',
              'Tello7',
              'Tello8',
              'Tello4',
              #'Tello8',
             ]


swarm = SWARM(wifi_interfaces, droneslist)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()

# =======================================================
# Initialize the PID controllers

drone1PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#---------------------------------------------------------------
drone2PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone2PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#--------------------------------------------------------------
drone3PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone3PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone3PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
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

#drone5PIDx = PID(Kp=300, Kd=300, Ki=0.0,
#                  derivativeFilterFreq=15,
#                  minOutput = -70, maxOutput = 70,
#                  current_time = None)
#
#drone5PIDy = PID(Kp=400, Kd=170, Ki=0.0,
#                  derivativeFilterFreq=15,
#                  minOutput = -70, maxOutput = 70,
#                  current_time = None)
#
#drone5PIDz = PID(Kp=300, Kd=300, Ki=0.0,
#                  derivativeFilterFreq=15,
#                  minOutput = -70, maxOutput = 70,
#                  current_time = None)

#--------------------------------------------------------------

for drone in allDrones:   
    drone.connect()  

for drone in allDrones:
    drone.takeoff()

rosClock.sleepForRate(10000)

i = 0
j = 1
Ts = 0.02 # sample time
theta = 1


pos_curr = []
for drone in GroundTruth:
    pos = drone.getPose()[0]
    #print("pos of drone is: ",pos)
    pos_curr.append(pos)
print("Initial_pos: ", pos_curr)
goal_pos = pos_curr

try: 
    while j < 3600:
        
        location = path_planning.Hover(pos_curr, goal_pos)
        
        print("--------------------------------")
        pos_curr = []
        for drone in GroundTruth:
            pos = drone.getPose()[0]
            #print("pos of drone is: ",pos)
            pos_curr.append(pos)
        print("Current Posotion: ", pos_curr)    
        
        
        if j > 600 and j < 1500:
            
            location = Character.form_I(pos_curr, [0.4, 0, 0.4], min_dist = [0.55,0.55,0.4])

        if j > 1499 and j < 2500:
            
            location = Character.form_Square(pos_curr, [0.2,-0.2,0.5])

        if j >2499 and j < 3200:

            location = Character.form_L(pos_curr, [0.3, -0.2, 0.5], min_dist = [0.6,0.6,0.4])
        
        print("location: ", location)  
        
        
        
        PIDvx1 = drone1PIDx.update(pos_curr[0][0], location[0][0])
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], location[0][1])
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], location[0][2])
        #print("DROne 1: ", PIDvx1, PIDvy1, PIDvz1)
        
        PIDvx2 = drone2PIDx.update(pos_curr[1][0], location[1][0])
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], location[1][1])
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], location[1][2])
        #print("DROne 2: ", PIDvx2, PIDvy2, PIDvz2)
        
        PIDvx3 = drone3PIDx.update(pos_curr[2][0], location[2][0])
        PIDvy3 = drone3PIDy.update(pos_curr[2][1], location[2][1])
        PIDvz3 = drone3PIDz.update(pos_curr[2][2], location[2][2])
        #print("DROne 3: ", PIDvx3, PIDvy3, PIDvz3)
    
        PIDvx4 = drone4PIDx.update(pos_curr[3][0], location[3][0])
        PIDvy4 = drone4PIDy.update(pos_curr[3][1], location[3][1])
        PIDvz4 = drone4PIDz.update(pos_curr[3][2], location[3][2])
        #print("DROne 4: ", PIDvx4, PIDvy4, PIDvz4)
        
        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
        allDrones[3].cmdVelocity(PIDvx4, PIDvy4, PIDvz4, 0)
        
        print(j)
        j+=1   
        rosClock.sleepForRate(1/Ts)           

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

















