#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Wed Jul 28 14:56:48 2021
@author: Mohammad
Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology
"""


import rospy
import tf
import tf2_ros
import numpy as np

import time



class rosClock:
    """
    Description ...
    
    this object was adopted from TimeHelper object in
    https://github.com/USC-ACTLab/crazyswarm
    
    """
    def __init__(self):
        self.rosRate = None
        self.rateHz = None

    def time(self):
        """Returns the current time in seconds."""
        return rospy.Time.now().to_sec()

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        rospy.sleep(duration)

    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        if self.rosRate is None or self.rateHz != rateHz:
            self.rosRate = rospy.Rate(rateHz)
            self.rateHz = rateHz
        self.rosRate.sleep()

    def isShutdown(self):
        """Returns true if the script should abort, e.g. from Ctrl-C."""
        return rospy.is_shutdown()
    

class MotionCapture:
    """
    
    TODO:
        make the world and child(object) frames configurable
        
        currently the default structure is consistent only with vicon_bridge pkg
    """
    
    WORLD_FRAME ='/vicon/world'
    
    
    def __init__(self, OBJECT_FRAME):
        rospy.init_node("GroundTruthPose", anonymous=False, disable_signals=True)
        self.tflistener = tf.TransformListener()
        self.OBJECT_FRAME = '/vicon'+str('/')+str(OBJECT_FRAME)+str('/')+str(OBJECT_FRAME)
        


    def getPose(self):
        
        error_count = 0
        while not rospy.is_shutdown() and error_count < 5:
            
            pos = [0, 0, 0]
            rot = [0, 0, 0]
                        
            try:
                self.tflistener.waitForTransform( MotionCapture.WORLD_FRAME,self.OBJECT_FRAME, rospy.Time(0), rospy.Duration(10))
                position, quaternion = self.tflistener.lookupTransform(MotionCapture.WORLD_FRAME, self.OBJECT_FRAME, rospy.Time(0))
                rotation = quaternion
                # in list format
                # TODO:
                
                # use tf_convert to get rotaion in Euler fromat
                # specify the units of measurement
                # specify the coordinate system 
                # print(position) 
                pos = np.array(position)
                rot = np.array(rotation)  
                #raise tf.LookupException()

                return pos, rot
                break

            except(tf2_ros.TransformException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):              

                print("Hello")
                print(error_count)
                error_count += 1
                if error_count > 3: 

                    print("Maximum Exceptions reached")
                    return pos,rot


                
    def testprint():
        print("Printing function")
 
        

if __name__ == '__main__':
    # rospy.init_node("GroundTruthPose", anonymous=False)
    try:
        OBJECT_FRAME = input("Enter the object name deinded in VICON Tracker: ") 
        print("Hello", OBJECT_FRAME + "!")
        Drone_GroundTruthPose = MotionCapture(str(OBJECT_FRAME))
        
        rate = rospy.Rate(1.0) # in Hz
        while not rospy.is_shutdown():
                    pos, rot = Drone_GroundTruthPose.getPose()
                    print("The Position and Rotation are:")
                    print(pos)
                    print(rot)
                    print('Press ctrl+C to stop...')
                    rate.sleep() 
                    
        # OBJECT_FRAME = input("Enter the object name deinded in VICON Tracker: ") 
        # print("Hello", OBJECT_FRAME + "!")
        # print("The Position and Rotation are:")
        # Drone_GroundTruthPose = MotionCapture(str(OBJECT_FRAME))
        # pos, rot = Drone_GroundTruthPose.getPose()
        # print(pos)
        # print(rot)
    # except rospy.ROSInterruptException:  pass
    except KeyboardInterrupt:  pass