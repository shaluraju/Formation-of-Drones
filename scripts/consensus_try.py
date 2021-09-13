#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 12 14:06:16 2021
@author: Mo
Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology
"""

from TelloServer import SWARM, Telloserver

# import time

from PID import PID

import numpy as np


"""
wlx9cefd5fb4bdd 192.168.10.2 Tello 1 (Tello-632C18) 5
wlx9cefd5faea28 192.168.10.3 Tello 2 (Tello-6272DB) 2
wlx9cefd5fae83e 192.168.10.4 Tello 3 (Tello-632C69) 4
wlx9cefd5fae837 192.168.10.5 Tello 4 (Tello-632C68) 3
wlx9cefd5faec05 192.168.10.x Tello 5 (Tello-632C38) x
wlx9cefd5fb6d84
wlx9cefd5fb6d84
"""
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

GrounTruth = swarm.MotionCaptureGroundTruth()

# =======================================================
# Initialize the PID controllers

drone1PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone1PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

# =========

drone2PIDx = PID(Kp=200, Kd=150, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone2PIDy = PID(Kp=400, Kd=170, Ki=0.0,
                  derivativeFilterFreq=20,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=130, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -100, maxOutput = 100,
                  current_time = None)

# =========

# drone3PIDx = PID(Kp=200, Kd=150, Ki=0.0,
#                   derivativeFilterFreq=20,
#                   minOutput = -100, maxOutput = 100,
#                   current_time = None)

# drone3PIDy = PID(Kp=400, Kd=170, Ki=0.0,
#                   derivativeFilterFreq=15,
#                   minOutput = -100, maxOutput = 100,
#                   current_time = None)

# drone3PIDz = PID(Kp=300, Kd=130, Ki=0.0,
#                   derivativeFilterFreq=15,
#                   minOutput = -100, maxOutput = 100,
#                   current_time = None)

# =========



# =======================================================

for drone in allDrones:   
    drone.connect()
  
for drone in allDrones:
    print(drone)   
    drone.takeoff()
    #It uses hardware redundacy (multiple Wi-Fi adapters) to establish unique 
       

# rosClock.sleep(.2)

# %% main code

# try:
#     while not rosClock.isShutdown():
#         # break (the main body of code)
# except KeyboardInterrupt:  pass


itr = 0
max_itr = 300
itr_sw = 300
Ts = 0.02 # sample time


DATA1 = np.zeros((max_itr,6))
DATA2 = np.zeros((max_itr,6))
DATA3 = np.zeros((max_itr,6))

DATAref = np.zeros((max_itr,9))
# ref1 = np.array([+1, +1, 0.7])
# ref2 = np.array([+1, -1, 0.8])
# ref3 = np.array([-1, -1, 1.3])
# ==================================


# Laplacian matrix
Lap = np.array([[1,-1,0], [-1,2,-1], [0,-1,1]])

alpha = 1;
# gamma = 2;

# forward Euler method

# 1st-order
# R+ = (I - alpha*Ts*Lap)*R
# R+ = (np.eye(len(droneslist)) - alpha*Ts*Lap).dot(R)

def traj_gen(pos, itr: int , itr_sw:int ):   
    
    if itr < itr_sw:
        
        ref1 = np.array([0, 0, 0.8])
        ref2 = np.array([1, 1, 0.8])
        
    # else:
        
    #     # R+ = (I - alpha*Ts*Lap)*R
    #     ref = (np.eye(len(droneslist)) - 0.1*alpha*Ts*Lap).dot(pos)
    #     # ref = np.array([.91, .8, 1.3])
        
    # ref1 = np.array([+1, +1, ref[0]])
    # ref2 = np.array([+1, -1, ref[1]])
    # ref3 = np.array([-1, -1, ref[2]])
        
    return ref1, ref2 #, ref2, ref3

# High-level consensus/formation => pos_ref (this part varies, depending coord. task)
# mid-level PID controller       => vel_ref (this part varies, depending on drones dynamics)
# low-level drone controller     => follows vel_ref to get pos_ref
# ==================================

try:
    # while not rosClock.isShutdown() and itr < max_itr:
    while itr < max_itr:    
        
        # =============== update data ====================
        # for i in range(len(droneslist)):    
        #     pos, _ = GrounTruth[i].getPose()
        #     print('position of Tello', i+1,':', pos)
        
        pos_curr = []
        for drone in GrounTruth:   
            pos = drone.getPose()[0]
            print(pos)
            pos_curr.append(pos)
            
        DATA1[itr,0:3] = pos_curr[0]
        DATA2[itr,0:3] = pos_curr[1] 
        # DATA3[itr,0:3] = pos_curr[2] 
        # ================================================   
        #ref1,ref2 = traj_gen(np.array([round(pos_curr[0][2],2)]),
                                              #round(pos_curr[1][2],2),
                                              #round(pos_curr[2][2],2)]),
                                    #itr=itr , itr_sw = itr_sw)

        #DATAref[itr,0:3] = ref1
        #DATAref[itr,3:6] = ref2
        #DATAref[itr,6:9] = ref3
        
        # =========== update controllers =================
        # print('pos_curr:', pos_curr)
        # print('pos_curr[0]:', pos_curr[0])
        # print('pos_curr[0][1]:', pos_curr[0][1])
        # for i in range(3):
        #     drone1PIDx.update(pos_curr[0][i], ref1[i])
        

        PIDvx1 = drone1PIDx.update(pos_curr[0][0], 0.0)
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], 0.0)
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], 0.8)
        

        PIDvx2 = drone2PIDx.update(pos_curr[1][0], 1.0)
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], 1.0)
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], 0.8)
        
        # PIDvx3 = drone3PIDx.update(pos_curr[2][0], ref3[0])
        # PIDvy3 = drone3PIDy.update(pos_curr[2][1], ref3[1]) 
        # PIDvz3 = drone3PIDz.update(pos_curr[2][2], ref3[2])
        
        
        DATA1[itr,3:6] = [PIDvx1, PIDvy1, PIDvz1]
        DATA2[itr,3:6] = [PIDvx2, PIDvy2, PIDvz2]
        # DATA3[itr,3:6] = [PIDvx3, PIDvy3, PIDvz3]
        
        print('PIDv1:', PIDvx1, PIDvy1, PIDvz1)
        print('PIDv2:', PIDvx2, PIDvy2, PIDvz2)
        
        #     drone2.cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        # ================================================ 
        
        # ============= send control commands ============
        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        # allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)

        
        # for drone in allDrones:   
        #     drone.cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
         
        # ================================================ 
        
        # rosClock.sleep(Ts)
        rosClock.sleepForRate(1/Ts)
        itr = itr+1
        
except KeyboardInterrupt:
    
    print('emergency interruption!; aborting all flights ...')
    
    for i in range(2):
        for drone in allDrones:
            # drone.land()
            drone.emergency()
    
        # rosClock.sleep(.1)
        rosClock.sleepForRate(10)
        
    np.save("testDATA",DATA1)
    # np.save("testDATA2",DATA2)
    # np.save("testDATA3",DATA3) 
    np.save("testDATAref",DATAref)
    pass

else:

    Telloserver.Landing(GrounTruth, allDrones)

          # for i in range(70):
    #     for drone in allDrones: 
    # #         drone.land()
    # #     rosClock.sleepForRate(10)
   
    # for i in range(50):
    #     for drone in allDrones:     
    #         drone.land()
    #     time.sleep(.1)    
    
    # rosClock.sleep(1)

    # for drone in allDrones:   
    #     drone.Disconnect()
    
    # np.save("testDATA",DATA1)
    # # np.save("testDATA2",DATA2)
    # # np.save("testDATA3",DATA3) 
    # np.save("testDATAref",DATAref)     
     


    

# %% 

    
# for i in range(100):
#     for drone in allDrones:     
#         drone.land()
#     time.sleep(.1) 
   
# for drone in allDrones:   
#     drone.Disconnect()
