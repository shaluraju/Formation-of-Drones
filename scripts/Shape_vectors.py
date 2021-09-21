# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 12:03:07 2021

@author: shalu
"""

class LineFormation:
    
    def form(min_dist_x: float, min_dist_y: float, min_dist_z: float, uav_pos: list):
        
        error = []
        #print("No of Drones: ", len(uav_pos))
        # first drone is used to drive
        for i in range(len(uav_pos) - 1):
            
            uav_x = uav_pos[0][0] - uav_pos[i+1][0] + min_dist_x*(i+1)
            uav_y = uav_pos[0][1] - uav_pos[i+1][1] + min_dist_y*(i+1)
            uav_z = uav_pos[0][2] - uav_pos[i+1][2] + min_dist_z*(i+1)
            
            error.append([uav_x, uav_y, uav_z])
        return error
