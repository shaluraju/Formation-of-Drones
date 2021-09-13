#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 28 17:15:47 2021

@author: Mo

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

from mocap import  MotionCapture as GroundTruth

import time
# replace time with rospy.time

    
OBJECT_Name = 'Tello5' # the object name definded in VICON Tracker


# pos, rot = GroundTruth(OBJECT_Name).getPose()
# print(pos)

Drone_GroundTruthPose = GroundTruth(OBJECT_Name)


i = 0

    
try:
    for i in range(50):
        pos, rot = Drone_GroundTruthPose.getPose()
        print(i)
        print(pos)
        time.sleep(1)


except KeyboardInterrupt:
    print("Interrupted")

        
    
