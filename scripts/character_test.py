#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  2 19:52:21 2021

@author: shalu
"""

from TelloServer import SWARM, Telloserver
import numpy as np
from Shape_vectors import LineFormation
import time
from PID import PID
import matplotlib.pyplot as plt

wifi_interfaces = ["wlx9cefd5fb774c",
                   "wlx9cefd5fb6d70",
                    "wls1",
                   #"wlx9cefd5fae83e"
                   ]

droneslist = ['Tello4',
              'Tello2',
              'Tello1'
             ]


swarm = SWARM(wifi_interfaces, droneslist)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()


for drone in allDrones:
    drone.connect()
    
for drone in allDrones:
    drone.takeoff()
    
rosClock.sleepForRate(1000)

drone1PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone1PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)


drone2PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone2PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)


drone3PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone3PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone3PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone4PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone4PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone4PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)


drone5PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone5PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone5PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)


drone6PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone6PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone6PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)



i = 0
Ts = 0.2


pos = []
#print("i: ", i)
#print("Goal Position given: ", leader_traj[i])
for drone in GroundTruth:   
    pos_ = drone.getPose()[0]
    #print("pos of drone is: ",pos)
    pos.append(pos_)

while i < 500:
    
        pos_curr = []
        #print("i: ", i)
        #print("Goal Position given: ", leader_traj[i])
        for drone in GroundTruth:   
            pos = drone.getPose()[0]
            #print("pos of drone is: ",pos)
            pos_curr.append(pos)
   
        PIDvx1 = drone1PIDx.update(pos_curr[0][0], pos[0][0])
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], pos[0][1])
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], pos[0][2])

        PIDvx2 = drone2PIDx.update(pos_curr[1][0], pos[1][0])
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], pos[1][1])
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], pos[1][2])
        
        PIDvx3 = drone3PIDx.update(pos_curr[2][0], pos[2][0])
        PIDvy3 = drone3PIDy.update(pos_curr[2][1], pos[2][1]) 
        PIDvz3 = drone3PIDz.update(pos_curr[2][2], pos[2][2])      
    
    