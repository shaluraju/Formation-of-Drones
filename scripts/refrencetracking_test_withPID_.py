#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug  6 08:52:42 2021

@author: Mo

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

# DO NOT FORGET source devel/setup.bash
# add check frame loss in the controller!! otherwise it uses wrong data and crashes 
# get yaw from mocap and use in control to keep it 0
# how crazyros uses one marker for tracking?
# go to emergency state when vicon mocap is not available

from mocap import MotionCapture 

from TelloServer import  Telloserver

from PID import PID

import time
import numpy as np
drone1 = Telloserver(wifi_interface="wlx9cefd5faea28")

#======================
drone1PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                 derivativeFilterFreq=20,
                 minOutput = -100, maxOutput = 100,
                 current_time = None)

drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                 derivativeFilterFreq=20,
                 minOutput = -100, maxOutput = 100,
                 current_time = None)

drone1PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                 derivativeFilterFreq=20,
                 minOutput = -100, maxOutput = 100,
                 current_time = None)
#=====================

OBJECT_Name = 'Tello6' # the drone name defined in VICON Tracker
Drone1_GroundTruth = MotionCapture(OBJECT_Name)

drone1.connect()
drone1.takeoff()

ref = np.array([0, 0, 1])

i = 0
i_max = 3000
Ts = 0.01

DATA = np.zeros((i_max,6))

     
for i in range(i_max):
    
    pos, rot = Drone1_GroundTruth.getPose()
    
    e = ref - pos
    print('sample:', i, 'tracking error:', e)
    DATA[i,0:3] = pos
     
    PIDvx = drone1PIDx.update(pos[0], ref[0])
    PIDvy = drone1PIDy.update(pos[1], ref[1])
    PIDvz = drone1PIDz.update(pos[2], ref[2])
    DATA[i,3:6] = [PIDvx, PIDvy, PIDvz]
    print('PIDv:', PIDvx, PIDvy, PIDvz)
    drone1.cmdVelocity(PIDvx, PIDvy, PIDvz, 0)
    time.sleep(Ts)
    i = i+1
    
    
# time.sleep(1)
drone1.connect()
drone1.land()
drone1.Disconnect()

np.save("testDATA",DATA)