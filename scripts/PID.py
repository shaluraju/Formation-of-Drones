#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  5 17:51:22 2021

@author: Mo

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

import time


class PID:
    """
    PID controller with a lowpass filer on D term.
    approximations use Backward Euler method.
    
    TODO: I term
    
    """
    def __init__(self, Kp: float, Kd: float, Ki: float,
                 derivativeFilterFreq: int,
                 # previousTime: None,
                 minOutput: float, maxOutput: float,
                 current_time = None):
        """
        TODO:
        

        Parameters
        ----------
        Kp : float
            DESCRIPTION.
        Kd : float
            DESCRIPTION.
        Ki : float
            DESCRIPTION.
        derivativeFilterFreq : int
            DESCRIPTION.
        previousTime : float
            DESCRIPTION.
        minOutput : TYPE, optional
            DESCRIPTION. The default is None.
        maxOutput : TYPE, optional
            DESCRIPTION. The default is None.
        current_time : TYPE, optional
            DESCRIPTION. The default is None.

        Returns
        -------
        None.

        """
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.F = derivativeFilterFreq
        # self.sample_time = sampleTime
        self.Ki = Ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.previousError = 0.0
        self.previousDterm = 0.0
        self.currentTime = current_time if current_time is not None else time.time()
        # self.previousTime = ros::Time::now()
        self.previousTime = self.currentTime
        
        
        # self.reset(self)
        
    def reset(self):
        self.previousError = 0.0
        self.previousDterm = 0.0
        # self.previousTime = ros::Time::now()
        self.previousTime = time.time()


    def update(self, feedback_value, target_value, tracking_error=None, current_time=None):
        """
        

        Parameters
        ----------
        feedback_value : TYPE
            DESCRIPTION.
        target_value : TYPE
            DESCRIPTION.
        tracking_error : TYPE, optional
            DESCRIPTION. The default is None.
        current_time : TYPE, optional
            DESCRIPTION. The default is None.

        Returns
        -------
        TYPE
            DESCRIPTION.

        """

        
        
        # TOO: add if {dt>0 OR Ts} for updating the D term
        # what if feedback_value, target_value were not given
        control_parameteres = []
        for i in range(len(target_value)):    
            
            current_time = current_time if current_time is not None else time.time()
            error = (target_value[i] - feedback_value[i]) if tracking_error is None else tracking_error
            dt = current_time - self.previousTime
            
            Pterm = self.Kp * error
            Dterm = self.previousDterm/(1+dt*self.F) + (
                      self.Kd*self.F*(error - self.previousError))/(1+self.F*dt)
            Iterm = self.Ki * 0.0
            self.current_time = current_time if current_time is not None else time.time()
            self.previousError = error
            self.previousDterm = Dterm
            self.previousTime  = current_time
            output = Pterm + Dterm + Iterm
            control_parameteres.append(max(self.minOutput, min(self.maxOutput, output)))
        return control_parameteres
        
        

class DronePID:

        def __init__(self, minOutput: float, maxOutput: float, derivativeFilterFreq: int, PID_X: list,
                     PID_Y: list, PID_Z: list, current_time = None):

            self.derivativeFilterFreq = derivativeFilterFreq
            self.minOutput = minOutput
            self.maxOutput = maxOutput
            self.currentTime = current_time
            self.PID_X = PID_X
            self.PID_Y = PID_Y
            self.PID_Z = PID_Z
            
            self.drone = []
           
            self.drone.append(PID(PID_X[0], PID_X[2], PID_X[1], derivativeFilterFreq, minOutput, maxOutput, current_time))
            self.drone.append(PID(PID_Y[0], PID_Y[2], PID_Y[1], derivativeFilterFreq, minOutput, maxOutput, current_time))
            self.drone.append(PID(PID_Y[0], PID_Y[2], PID_Y[1], derivativeFilterFreq, minOutput, maxOutput, current_time))


        def link(self):
            print("linked")
            return self.drone


        def UPDATE(self, current: list, goal: list ) -> list:
            for direction in self.drone:
                return direction.update(current, goal)

            
            

