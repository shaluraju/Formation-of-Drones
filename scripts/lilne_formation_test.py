
"""
Created on Mon Sep 20 10:48:53 2021

@author: shalu
"""

from TelloServer import SWARM, Telloserver
import numpy as np
from Shape_vectors import LineFormation
from PID import DronePID 
# Connection
#----------------------------------------------------------------------------
wifi_interfaces = ["wlx9cefd5fb6d70",
                    "wlx9cefd5fb6d84"
                    #"wlx9cefd5faea28",
                   #"wlx9cefd5fae83e"
                   ]

droneslist = ['Tello6',
              'Tello5'
              #'Tello3'
             ]


swarm = SWARM(wifi_interfaces, droneslist)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()
for drone in allDrones:   
    drone.connect()
    drone.takeoff()

# PID Commands 

UAV_1 = DronePID(minOutput = -100, maxOutput = 100, derivativeFilterFreq=15,
                     PID_X = [200,0,150], PID_Y = [400,0,170], PID_Z = [300,0,130], current_time = None)

UAV_2 = DronePID(minOutput = -100, maxOutput = 100, derivativeFilterFreq=15,
                     PID_X = [200,0,150], PID_Y = [400,0,170], PID_Z = [300,0,130], current_time = None)

UAV_3 = DronePID(minOutput = -100, maxOutput = 100, derivativeFilterFreq=15,
                     PID_X = [200,0,150], PID_Y = [400,0,170], PID_Z = [300,0,130], current_time = None)

# For Drone 1
present = [2,10,10]
goal = [10,-20,12]


# Take Off
#----------------------------------------------------------------------------
# Takes initial position and gives take off upto giveh height


#UAV_1.UPDATE([1,-2,0], [1,-2,0.8])
#UAV_2.UPDATE([2,2,0], [2,2,0.8])
#UAV_3.UPDATE([2,-2,0], [2,-2,0.8])


#Line Formation
i = True
try:
    cur_pos = []
    
    while i :
        for uav in GroundTruth:
            print(uav.getPose()[0])
            cur_pos.append(uav.getPose()[0])
        
        pos_error = LineFormation.form(0.2,0.2,0.2,cur_pos)
        
        
except KeyboardInterrupt:
    
    print("Stopped")
    i = False
    for drone in allDrones:
        drone.land()

        
for drone in allDrones:
    drone.Disconnect()











