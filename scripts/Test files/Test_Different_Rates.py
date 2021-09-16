# -*- coding: utf-8 -*-
"""
Created on Tue Sep  7 22:42:46 2021

@author: sssha
"""

import rospy
import time

itr = 0
max_itr = 300
itr_sw = 300
Ts = 0.02 # sample time


def traj_gen(pos, itr: int , itr_sw:int ):   
    
    if itr < itr_sw:
        
        ref1 = np.array([0, 0, 1])
        ref2 = np.array([1, 1, 0.8])
        #print("Time is Now", rospy.Time.now()
        itr = itr+1

"""
        while not rosClock.isShutdown():
            try:
                # while not rosClock.isShutdown() and itr < max_itr:
                while itr < max_itr:    
                    drone_1 = DronePID(minOutput = -100, maxOutput = 100, derivativeFilterFreq=15,
                             PID_X = [200,0,150], PID_Y = [400,0,170], PID_Z = [300,0,130], current_time = None)
                    
                    present = [1,1,1]
                    goal = [2,2,2]
                    
                    #print("For Drone 1")
                    controls = drone_1.UPDATE(present, goal) 
                    print("PID Controls are: "controls)
                    
                    rate = rospy.rate(10)
                    different_rate()
                    rate.sleep()
                    itr = itr+1
                    
            except KeyboardInterrupt:
                
                print('emergency interruption!; aborting all flights ...')
                
                for i in range(2):
                    for drone in allDrones:
                        # drone.land()
                        drone.emergency()
                
                    # rosClock.sleep(.1)
                    rosClock.sleepForRate(10)
                pass
"""

def different_rate():
    print("Time in different rate is Now", rospy.Time.now())
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          