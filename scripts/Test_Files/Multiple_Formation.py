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
"""
Tello Drones are connected to wifi dongles thus, their interface
id's are defined here so that, the control commands are sent to
correct drone.  
"""

wifi_interfaces = ["wlx9cefd5fb6d70",
                   "wlx9cefd5fb71cc",
                   "wlx9cefd5fb774c",
                    "wlx9cefd5fb6a56",
                   "wlx9cefd5fae98b",
                   "wlx9cefd5fb6d84",
                   "wlx9cefd5faec05",
                   ]

droneslist = ['Tello3',
              'Tello4',
              'Tello5',
              'Tello6',
              'Tello8',
              'Tello9',
              'Tello10',
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

#------------------------------------------------        

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
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone5PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone5PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#---------------------------------------------------------------
drone6PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone6PIDy = PID(Kp=400, Kd=170, Ki=0.0        ,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone6PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#--------------------------------------------------------------
drone7PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone7PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone7PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=10,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

#---------------------------------------------------------------

#drone8PIDx = PID(Kp=300, Kd=300, Ki=0.0,
#                  derivativeFilterFreq=15,
#                  minOutput = -70, maxOutput = 70,
#                  current_time = None)
#
#drone8PIDy = PID(Kp=400, Kd=170, Ki=0.0,
#                  derivativeFilterFreq=15,
#                  minOutput = -70, maxOutput = 70,
#                  current_time = None)
#
#drone8PIDz = PID(Kp=300, Kd=300, Ki=0.0,
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
    while j < 2500:
        
        location = path_planning.Hover(pos_curr, goal_pos)
        
        print("--------------------------------")
        pos_curr = []
        for drone in GroundTruth:
            pos = drone.getPose()[0]
            #print("pos of drone is: ",pos)
            pos_curr.append(pos)
        print("Current Posotion: ", pos_curr)    
        
        
        if j > 600 and j < 1500:
            
            location = Character.form_S(pos_curr, [0.0, 0.0, 1.0])

        if j > 1499 and j < 2500:
            
            location = Character.form_A(pos_curr, [0.2,-0.2,0.5])


        
        print("Goal: ", location)  
        
        
        
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
        
        #---------------------------------------------------------
        
        PIDvx5 = drone5PIDx.update(pos_curr[4][0], location[4][0])
        PIDvy5 = drone5PIDy.update(pos_curr[4][1], location[4][1])
        PIDvz5 = drone5PIDz.update(pos_curr[4][2], location[4][2])
        #print("DROne 1: ", PIDvx1, PIDvy1, PIDvz1)
        
        PIDvx6 = drone6PIDx.update(pos_curr[5][0], location[5][0])
        PIDvy6 = drone6PIDy.update(pos_curr[5][1], location[5][1])
        PIDvz6 = drone6PIDz.update(pos_curr[5][2], location[5][2])
        #print("DROne 2: ", PIDvx2, PIDvy2, PIDvz2)
        
        PIDvx7 = drone7PIDx.update(pos_curr[6][0], location[6][0])
        PIDvy7 = drone7PIDy.update(pos_curr[6][1], location[6][1])
        PIDvz7 = drone7PIDz.update(pos_curr[6][2], location[6][2])
        #print("DROne 3: ", PIDvx3, PIDvy3, PIDvz3)
    
#        PIDvx8 = drone8PIDx.update(pos_curr[7][0], location[7][0])
#        PIDvy8 = drone8PIDy.update(pos_curr[7][1], location[7][1])
#        PIDvz8 = drone8PIDz.update(pos_curr[7][2], location[7][2])
        #print("DROne 4: ", PIDvx4, PIDvy4, PIDvz4)
        
        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
        allDrones[3].cmdVelocity(PIDvx4, PIDvy4, PIDvz4, 0)
        
        allDrones[4].cmdVelocity(PIDvx5, PIDvy5, PIDvz5, 0)
        allDrones[5].cmdVelocity(PIDvx6, PIDvy6, PIDvz6, 0)
        allDrones[6].cmdVelocity(PIDvx7, PIDvy7, PIDvz7, 0)
#        allDrones[7].cmdVelocity(PIDvx8, PIDvy8, PIDvz8, 0)
#        
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

















