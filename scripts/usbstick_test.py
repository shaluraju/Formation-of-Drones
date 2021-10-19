# -*- coding: utf-8 -*-
"""
Created on Fri Oct  8 14:17:33 2021

@author: shalu
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

# =========

drone2PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone2PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=20,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -70, maxOutput = 70,
                  current_time = None)

# =========

drone3PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                   derivativeFilterFreq=20,
                   minOutput = -70, maxOutput = 70,
                   current_time = None)

drone3PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                   derivativeFilterFreq=15,
                   minOutput = -70, maxOutput = 70,
                   current_time = None)

drone3PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                   derivativeFilterFreq=15,
                   minOutput = -70, maxOutput = 70,
                   current_time = None)

# =======================================================

for drone in allDrones:   
    drone.connect()  
    drone.takeoff()


rosClock.sleepForRate(5000)


for drone in allDrones:   
    drone.land()
    drone.Disconnect()

