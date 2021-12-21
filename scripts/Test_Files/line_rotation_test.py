# -*- coding: utf-8 -*-
"""
Created on Wed Oct 20 10:20:41 2021

@author: Shalu
"""
from TelloServer import SWARM, Telloserver
import numpy as np
from Shape_vectors import LineFormation
import time
from PID import PID
import matplotlib.pyplot as plt
#import timeit

"""
Tello Drones are connected to wifi dongles thus, their interface
id's are defined here so that, the control commands are sent to
correct drone.  
"""

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

# =======================================================
# Initialize the PID controllers

drone1PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

drone1PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

drone1PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)


drone2PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

drone2PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

drone2PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)


drone3PIDx = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

drone3PIDy = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

drone3PIDz = PID(Kp=300, Kd=300, Ki=0.0,
                  derivativeFilterFreq=15,
                  minOutput = -80, maxOutput = 80,
                  current_time = None)

for drone in allDrones:   
    drone.connect()  

for drone in allDrones:
    drone.takeoff()

rosClock.sleepForRate(3000)

i = 0
j = 0
Ts = 0.015 # sample time
theta = 1


# ------------- Line Formation ------------------

try:
    while j < 1000: 
        
        pos_curr = []
        for drone in GroundTruth:   
            pos = drone.getPose()[0]
            print("pos of drone is: ",pos)
            pos_curr.append(pos)
        
        PIDvx1 = drone1PIDx.update(pos_curr[0][0], 0.0)
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], -0.6)
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], 0.7)
        
        PIDvx2 = drone2PIDx.update(pos_curr[1][0], 0.0)
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], 0.0)
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], 0.7)
        
        PIDvx3 = drone3PIDx.update(pos_curr[2][0], 0.0)
        PIDvy3 = drone3PIDy.update(pos_curr[2][1], 0.6) 
        PIDvz3 = drone3PIDz.update(pos_curr[2][2], 0.7)
#        
        #print("PID: ", PIDvx1, PIDvy1,PIDvz1)
#        error = LineFormation.form(0.0,0.55,0.0, pos_curr)
#        print("error: ", error)
#        goal = []
#        for i in range(0,len(error)):
#            x = pos_curr[i+1][0] + error[i][0]
#            y = pos_curr[i+1][1] + error[i][1]
#            z = pos_curr[i+1][2] + error[i][2]
#            goal.append([x,y,z])
#        #print("Iteration: ", j)
#        print("goal: ", goal)
#        print("-----------------------------")        
#        PIDvx2 = drone2PIDx.update(pos_curr[1][0], goal[0][0])
#        PIDvy2 = drone2PIDy.update(pos_curr[1][1], goal[0][1])
#        PIDvz2 = drone2PIDz.update(pos_curr[1][2], goal[0][2])
#        
#        PIDvx3 = drone3PIDx.update(pos_curr[2][0], goal[1][0])
#        PIDvy3 = drone3PIDy.update(pos_curr[2][1], goal[1][1]) 
#        PIDvz3 = drone3PIDz.update(pos_curr[2][2], goal[1][2])
#        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
        j+=1
#        #print("Position : ", pos_curr)
        rosClock.sleepForRate(1/Ts)
#    
#    #rosClock.sleepForRate(1000)

 #------------------------    Rotation -----------------------------
    pos_curr = []
    for drone in GroundTruth:   
        pos = drone.getPose()[0]
        print("pos of drone is: ",pos)
        pos_curr.append(pos)
    
    leader_traj = LineFormation.full_rotation(pos_curr[0], 0.6)
    follower = LineFormation.full_rotation(pos_curr[1], -0.6)
    j = 0
    i = 0 
    while j < 3:
        print("Entered Rotation")
        for i in range(len(leader_traj)):
            pos_curr = []
            #print("i: ", i)
            #print("Goal Position given: ", leader_traj[i])
            for drone in GroundTruth:   
                pos = drone.getPose()[0]
                #print("pos of drone is: ",pos)
                pos_curr.append(pos)
       
            PIDvx1 = drone1PIDx.update(pos_curr[0][0], leader_traj[i][0])
            PIDvy1 = drone1PIDy.update(pos_curr[0][1], leader_traj[i][1])
            PIDvz1 = drone1PIDz.update(pos_curr[0][2], leader_traj[i][2])

            PIDvx2 = drone2PIDx.update(pos_curr[1][0], 0.0)
            PIDvy2 = drone2PIDy.update(pos_curr[1][1], 0.0) 
            PIDvz2 = drone2PIDz.update(pos_curr[1][2], 0.7) 

            PIDvx3 = drone3PIDx.update(pos_curr[2][0], follower[i][0])
            PIDvy3 = drone3PIDy.update(pos_curr[2][1], follower[i][1])
            PIDvz3 = drone3PIDz.update(pos_curr[2][2], follower[i][2])
            
                
            #print('PIDv1:', PIDvx1, PIDvy1, PIDvz1)
            allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
            allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
            allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
            #lt.plot(pos_curr[0][0],pos_curr[0][1],'ro')
            rosClock.sleepForRate(1/Ts)
        j+=1
        
#--------------------      Line Formation    ---------------------------------   
    i = 0
    while i < 500: 
    
        pos_curr = []
        for drone in GroundTruth:   
            pos = drone.getPose()[0]
            #print("pos of drone is: ",pos)
            pos_curr.append(pos)
        
        PIDvx1 = drone1PIDx.update(pos_curr[0][0], 0.0)
        PIDvy1 = drone1PIDy.update(pos_curr[0][1], -0.6)
        PIDvz1 = drone1PIDz.update(pos_curr[0][2], 0.7)
        
        PIDvx2 = drone2PIDx.update(pos_curr[1][0], 0.0)
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], 0.0)
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], 0.7)
        
        PIDvx3 = drone3PIDx.update(pos_curr[2][0], 0.0)
        PIDvy3 = drone3PIDy.update(pos_curr[2][1], 0.6) 
        PIDvz3 = drone3PIDz.update(pos_curr[2][2], 0.7)
        
        #print("PID: ", PIDvx1, PIDvy1,PIDvz1)
        # error = LineFormation.form(0.0, 0.55, 0.0, pos_curr)
        # goal = []
        # for i in range(len(error)):
        #     x = pos_curr[i+1][0] + error[i][0]
        #     y = pos_curr[i+1][1] + error[i][1]
        #     z = pos_curr[i+1][2] + error[i][2]
        #     goal.append([x,y,z])
        # print("goal: ", goal)
                
        # PIDvx2 = drone2PIDx.update(pos_curr[1][0], goal[0][0])
        # PIDvy2 = drone2PIDy.update(pos_curr[1][1], goal[0][1])
        # PIDvz2 = drone2PIDz.update(pos_curr[1][2], goal[0][2])
        
        # PIDvx3 = drone3PIDx.update(pos_curr[2][0], goal[1][0])
        # PIDvy3 = drone3PIDy.update(pos_curr[2][1], goal[1][1]) 
        # PIDvz3 = drone3PIDz.update(pos_curr[2][2], goal[1][2])
        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
        i += 1
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

#plt.savefig('/home/sas-lab/catkin_ws/src/vicon_bridge/scripts/line_rotation_plots/1_try.png') 
