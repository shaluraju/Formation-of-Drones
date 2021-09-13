#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 29 10:51:00 2021


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
# drone1 = Telloserver(wifi_interface="wlx9cefd5faea28")
# drone2 = Telloserver(wifi_interface="wlx9cefd5fb4bdd")

OBJECT_Name = 'Tello6' # the drone name defined in VICON Tracker
Drone1_GroundTruth = MotionCapture(OBJECT_Name)
Drone2_GroundTruth = MotionCapture('Tello1')

#======================
# drone1PIDx = PID(Kp=200, Kd=150, Ki=0.0,
#                  derivativeFilterFreq=20,
#                  minOutput = -100, maxOutput = 100,
#                  current_time = None)

# drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
#                  derivativeFilterFreq=20,
#                  minOutput = -100, maxOutput = 100,
#                  current_time = None)

# drone1PIDz = PID(Kp=300, Kd=130, Ki=0.0,
#                  derivativeFilterFreq=20,
#                  minOutput = -100, maxOutput = 100,
#                  current_time = None)

# drone2PIDx = PID(Kp=200, Kd=150, Ki=0.0,
#                  derivativeFilterFreq=20,
#                  minOutput = -100, maxOutput = 100,
#                  current_time = None)

# drone2PIDy = PID(Kp=400, Kd=170, Ki=0.0,
#                  derivativeFilterFreq=20,
#                  minOutput = -100, maxOutput = 100,
#                  current_time = None)

# drone2PIDz = PID(Kp=300, Kd=130, Ki=0.0,
#                  derivativeFilterFreq=20,
#                  minOutput = -100, maxOutput = 100,
#                  current_time = None)
#=====================



# drone1.connect()
# drone2.connect()

# drone1.takeoff()
# drone2.takeoff()

# time.sleep(1)

# REF1 = np.array([.5, -1, 1.2])
# REF2 = np.array([-1, 1, 0.8])

# i = 0
# i_max = 3000
# Ts = 0.01

# DATA1 = np.zeros((i_max,6))
# DATA2 = np.zeros((i_max,6))

#=====================================
# pos1, _ = Drone1_GroundTruth.getPose()
# print('pos1:', pos1)
# print('tested passed')



# try:
#     while True:
#         for i in range(5):
#             print(i)
#             time.sleep(1)
#             # i =i+1
#         break
# except KeyboardInterrupt:
#     print("Press Ctrl-C to terminate while statement")
#     pass

# while True:
#     try:     
#         for i in range(5):
#             print(i)
#             time.sleep(1)
#             # i =i+1
#         # break
#     except KeyboardInterrupt:
        
#         print('it worked')
#         break
# print('it passed')
     
  
flag = True
itr = 1

for i in range(60):
    
    pos1, _ = Drone1_GroundTruth.getPose()
    # pos2, _ = Drone2_GroundTruth.getPose() 
    print('pos1:', i,pos1)
    time.sleep(1)

#=====================================

     
# for i in range(i_max):
    
   
#     pos1, rot1 = Drone1_GroundTruth.getPose()
#     pos2, rot2 = Drone2_GroundTruth.getPose()
    
#     if type(pos1) != int or type(pos1) != float:
#         pos1 = POS1_stored
     
#     if type(pos2) != int or type(pos2) != float:
#         pos2 = POS2_stored    
    
#     if i<1500:
#         ref1 = REF1
#         ref2 = REF2
#     else:
#         ref1[0] = pos2[0]
#         ref2[0] = pos1[0]
        

    
#     e1 = ref1 - pos1
#     print('sample:', i, 'tracking error1:', e1)
#     DATA1[i,0:3] = pos1
    
#     e2 = ref2 - pos2
#     print('sample:', i, 'tracking error2:', e2)
#     DATA2[i,0:3] = pos2
     
#     PIDvx1 = drone1PIDx.update(pos1[0], ref1[0])
#     PIDvy1 = drone1PIDy.update(pos1[1], ref1[1])
#     PIDvz1 = drone1PIDz.update(pos1[2], ref1[2])
#     DATA1[i,3:6] = [PIDvx1, PIDvy1, PIDvz1]
#     print('PIDv1:', PIDvx1, PIDvy1, PIDvz1)
#     drone1.cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
    
#     PIDvx2 = drone2PIDx.update(pos2[0], ref2[0])
#     PIDvy2 = drone2PIDy.update(pos2[1], ref2[1])
#     PIDvz2 = drone2PIDz.update(pos2[2], ref2[2])
#     DATA2[i,3:6] = [PIDvx2, PIDvy2, PIDvz2]
#     print('PIDv2:', PIDvx2, PIDvy2, PIDvz2)
#     drone2.cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
    
#     time.sleep(Ts)
#     i = i+1
    
    
# for i in range(5):
#     drone1.land()
#     drone2.land()
    

# drone1.Disconnect()
# drone2.Disconnect()

# np.save("testDATA",DATA1)
# np.save("testDATA2",DATA2)