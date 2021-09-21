# -*- coding: utf-8 -*-
"""
Created on Fri Sep 17 20:01:20 2021

@author: shalu
"""

import time


class newPID:
    """
    PID controller with a lowpass filer on D term.
    approximations use Backward Euler method.
    
    TODO: I term
    
    """
    def __init__(self, minOutput: float, maxOutput: float, derivativeFilterFreq: int, PID_X: list,
                     PID_Y: list, PID_Z: list, current_time = None):
    
        
        self.PID_X = PID_X
        self.PID_Y = PID_Y
        self.PID_Z = PID_Z
        #self.Kp = Kp
        #self.Kd = Kd
        #self.Ki = Ki
        self.F = derivativeFilterFreq
        # self.sample_time = sampleTime
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.previousError = 0.0
        self.previousDterm = 0.0
        self.currentTime = current_time if current_time is not None else time.time()
        # self.previousTime = ros::Time::now()
        self.previousTime = self.currentTime
        self.xyz = [PID_X, PID_Y,PID_Z]

        
    def reset(self):
        self.previousError = 0.0
        self.previousDterm = 0.0
        # self.previousTime = ros::Time::now()
        self.previousTime = time.time()


    def update(self, current_pos: list, goal_pos: list, tracking_error=None, current_time=None):       
        
        # TOO: add if {dt>0 OR Ts} for updating the D term
        # what if feedback_value, target_value were not given
        control_parameteres = []
        for i in range(len(self.xyz)):    
            
            current_time = current_time if current_time is not None else time.time()
            error = (goal_pos[i] - current_pos[i]) if tracking_error is None else tracking_error
            #dt = 0.00099778
            dt = current_time - self.previousTime
            print("dt = :", dt)
            Pterm = self.xyz[i][0] * error
            print("Pterm = :", Pterm)
            Dterm = self.previousDterm/(1+dt*self.F) + (
                      self.xyz[i][2]*self.F*(error - self.previousError))/(1+self.F*dt)
            print("Dterm = :", Dterm)
            Iterm = self.xyz[i][1] * 0.0
            print("Iterm = :", Iterm)
            self.current_time = current_time if current_time is not None else time.time()
            self.previousError = error
            self.previousDterm = Dterm
            self.previousTime  = current_time
            output = Pterm + Dterm + Iterm
            print("sumPID= :", output)
            control_parameteres.append(max(self.minOutput, min(self.maxOutput, output)))
        return control_parameteres
        
        

