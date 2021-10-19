

"""
Created on Mon Sep 20 10:48:53 2021

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


rosClock.sleepForRate(3000)

itr = 0
max_itr = 2000
itr_sw = 300
Ts = 0.02 # sample time

try:

    while itr < max_itr:    
        
        pos_curr = []
        for drone in GroundTruth:   
            pos = drone.getPose()[0]
            print("pos of drone is: ",pos)
            pos_curr.append(pos)
              
        
        error = LineFormation.form(0.5,0.5,0.0, pos_curr)
        goal = []
        for i in range(len(error)):
            x = pos_curr[i+1][0] + error[i][0]
            y = pos_curr[i+1][1] + error[i][1]
            z = pos_curr[i+1][2] + error[i][2]
            goal.append([x,y,z])
        print("goal: ", goal)
        
        if itr > 700 and itr < 1300:
            PIDvx1 = drone1PIDx.update(pos_curr[0][0], -1.0)
            PIDvy1 = drone1PIDy.update(pos_curr[0][1], 0.0)
            PIDvz1 = drone1PIDz.update(pos_curr[0][2], 0.8)
            
        elif itr < 700:    
            PIDvx1 = drone1PIDx.update(pos_curr[0][0], 0.0)
            PIDvy1 = drone1PIDy.update(pos_curr[0][1], 0.0)
            PIDvz1 = drone1PIDz.update(pos_curr[0][2], 0.8)
        
        elif itr > 1300:
            
            dt = max_itr - itr 
            l_pos = LineFormation.rotation(pos_curr, dt)
            PIDvx1 = drone1PIDx.update(pos_curr[0][0], l_pos[0])
            PIDvy1 = drone1PIDy.update(pos_curr[0][1], l_pos[1])
            PIDvz1 = drone1PIDz.update(pos_curr[0][2], l_pos[2])
        
            
        PIDvx2 = drone2PIDx.update(pos_curr[1][0], goal[0][0])
        PIDvy2 = drone2PIDy.update(pos_curr[1][1], goal[0][1])
        PIDvz2 = drone2PIDz.update(pos_curr[1][2], goal[0][2])
        
        PIDvx3 = drone3PIDx.update(pos_curr[2][0], goal[1][0])
        PIDvy3 = drone3PIDy.update(pos_curr[2][1], goal[1][1]) 
        PIDvz3 = drone3PIDz.update(pos_curr[2][2], goal[1][2])
        
        

        print('PIDv1:', PIDvx1, PIDvy1, PIDvz1)
        print('PIDv2:', PIDvx2, PIDvy2, PIDvz2)
        print('PIDv3:', PIDvx3, PIDvy3, PIDvz3)
        
        # ================================================ 

        
        allDrones[0].cmdVelocity(PIDvx1, PIDvy1, PIDvz1, 0)
        allDrones[1].cmdVelocity(PIDvx2, PIDvy2, PIDvz2, 0)
        allDrones[2].cmdVelocity(PIDvx3, PIDvy3, PIDvz3, 0)
        
        # rosClock.sleep(Ts)
        rosClock.sleepForRate(1/Ts)
        itr = itr+1
        
except KeyboardInterrupt:
    
    print('emergency interruption!; aborting all flights ...')   
    for i in range(2):
        for drone in allDrones:
            drone.emergency()
        rosClock.sleepForRate(10)      
    pass

else:

    for drone in allDrones:
        drone.land()

   
for drone in allDrones:   
     drone.Disconnect()






