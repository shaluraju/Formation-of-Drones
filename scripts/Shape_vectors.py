# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 12:03:07 2021

@author: shalu
"""

class LineFormation:
    
    def form(min_dist_x: float, min_dist_y: float, min_dist_z: float, uav_pos: list):
        
        # first drone is used to drive
        uav12_x = uav_pos[0][0] - uav_pos[1][0] - min_dist_x
        uav13_x = uav_pos[0][0] - uav_pos[2][0] - min_dist_x*2
        
        uav12_y = uav_pos[0][1] - uav_pos[1][1] - min_dist_y
        uav13_y = uav_pos[0][1] - uav_pos[2][1] - min_dist_y*2
        
        uav12_z = uav_pos[0][2] - uav_pos[1][2] - min_dist_z
        uav13_z = uav_pos[0][2] - uav_pos[2][2] - min_dist_z*2       
        
        return [[uav12_x,uav12_y,uav12_z],[uav13_x,uav13_y,uav13_z]]