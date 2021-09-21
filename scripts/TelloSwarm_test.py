#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  9 19:46:10 2021

@author: Mo

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

from TelloServer import SWARM

import time

wifi_interfaces = ["wlx9cefd5fb4bdd",
                   "wlx9cefd5faea28"]

droneslist = ['Tello1', 
              'Tello2']


swarm = SWARM(wifi_interfaces, droneslist)
# allDrones = swarm.allDrones

GrounTruth = swarm.MotionCaptureGroundTruth()

# print(type(GrounTruth))
# print(GrounTruth)
# pos = GrounTruth[0]
# print(pos)
# for i in range(len(droneslist)):   
#     allDrones[i].connect()
  
    
# for i in range(len(droneslist)):   
#     allDrones[i].takeoff()
    

# time.sleep(2)

for i in range(len(droneslist)):   
    pos, _ = GrounTruth[i].getPose()
    print('position of Tello', i+1,':', pos)
    
pos, _ = GrounTruth.getPose()
print('position of Tello',':', pos)
# %% 
# for i in range(len(droneslist)):   
#     allDrones[i].land()

# for i in range(100):
#     for i in range(len(droneslist)):   
#         allDrones[i].land()
#     time.sleep(.1)    