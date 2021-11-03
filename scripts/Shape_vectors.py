# -*- coding: utf-8 -*-
"""
Created on Mon Sep 20 12:03:07 2021

@author: shalu
"""
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d 

class LineFormation:
    
    
    def checkboundary(leader_goal: list):
        x_limits = [-1.8,1.8]
        y_limits = [-2.8,2.8]
        z_limits = [0.3,2.3]
        
        l = leader_goal 
        
        if ( (l[0] < x_limits[0] or l[0] > x_limits[1]) or
             (l[1] < y_limits[0] or l[1] > y_limits[1]) or 
             (l[2] < z_limits[0] or l[2] > z_limits[1]) ):
            return False
        else:
            return True
        
    
    def form(min_dist_x: float, min_dist_y: float, min_dist_z: float, uav_pos: list) -> list:
        

        error = []
        #print("No of Drones: ", len(uav_pos))
        # first drone is used to drive
        
        number_of_drones = len(uav_pos)
        #print(number_of_drones)
        x = uav_pos[0][0] + (number_of_drones-1) * min_dist_x 
        y = uav_pos[0][1] + (number_of_drones-1) * min_dist_y
        z = uav_pos[0][2] + (number_of_drones-1) * min_dist_z
    
        #print(x,y,z)    
        #go_ahead = LineFormation.checkboundary([x,y,z])
        go_ahead = True
        
        if go_ahead == False:
            
            print("The Formation is gonna hit the Walls")
            print("Choose within the boundary limits, flight area is about 4x6x2.5 m^3")
            
        elif go_ahead == True: 
            
            for i in range(len(uav_pos) - 1):
                
                uav_x = uav_pos[0][0] - uav_pos[i+1][0] + min_dist_x*(i+1)
                uav_y = uav_pos[0][1] - uav_pos[i+1][1] + min_dist_y*(i+1)
                uav_z = uav_pos[0][2] - uav_pos[i+1][2] + min_dist_z*(i+1)
                
                error.append([uav_x, uav_y, uav_z])
            return error
    
    
    def rotation(uav_pos: list, min_dist: float):
        # Returs the position of the leader drone on circumference
        midpoint = uav_pos[1] 
        radius = min_dist
        
        original_dist = math.sqrt( (midpoint[0]-uav_pos[0][0])**2 + (midpoint[1]-uav_pos[0][1])**2 )
        print("Original Dist: ", original_dist)
        new_pos_x = uav_pos[0][0] + 0.1
        new_pos_y = uav_pos[0][1] + 0.1
        
        print("new_pos_x = ",new_pos_x )
        print("new_pos_y = ",new_pos_y )
        
        # Dist from Center
        dist = math.sqrt( (midpoint[0]-new_pos_x)**2 + (midpoint[1]-new_pos_y)**2 )
        print("adjacent: ", dist)

        dist_bw_pos = math.sqrt( 2*0.1**2 )
        
        print("step Dist: ", dist_bw_pos)
        theta = math.atan(dist_bw_pos/dist)
        print("theta: ", theta)
        relative_dist = min_dist - dist
        print("relative dist: ", relative_dist)

        add_x = math.sin(theta)*relative_dist
        add_y = math.cos(theta)*relative_dist
        
        print("after adding: ",new_pos_x+add_x)
        print("after addinng: ",new_pos_y+add_y)
        
        return [new_pos_x + add_x, new_pos_y + add_y, uav_pos[0][2]]
    
    def full_rotation(uav_pos: list, min_dist: float):
        
        
    # This method returns the trajectory of leader during the full rotation
    # i.e 360 degrees, given the starting position of all the drones
        theta = 1
        start_pos = uav_pos
        print("leader Pos: ",start_pos)
        traj = []
        while theta < 512:    
        #print(theta)
            tta = round(theta/80, 3)
            #print("theta: ", theta)
            #print(tta)
            add_x = round( math.sin(tta)*min_dist, 2)
            #print(add_x)
            add_y = round( math.cos(tta)*min_dist, 2)  
            #print(add_y)
            
            if tta < 1.5:   

                leader_pos = [start_pos[0] + add_x, start_pos[1] + min_dist - add_y, start_pos[2]]
                #print('1',leader_pos)
                traj.append(leader_pos)
            
            elif tta > 1.4 and tta < 3.1:

                leader_pos = [start_pos[0] + add_x, start_pos[1] + min_dist - add_y, start_pos[2]]
                #print('2',leader_pos)
                traj.append(leader_pos)
    
            elif tta > 3.0 and tta < 4.7:

                leader_pos = [start_pos[0] + add_x, start_pos[1] + min_dist - add_y, start_pos[2]]
                #print('3',leader_pos)
                traj.append(leader_pos)
            
            elif tta > 4.6 and tta < 6.4:

                leader_pos = [start_pos[0] + add_x, start_pos[1] + min_dist - add_y, start_pos[2]]
                #print('4',leader_pos)
                traj.append(leader_pos)
            
            theta += 1
        return traj
          
        

        
        
class Character:
    
    # This class is used to make a formation in the shape of characters
    
    def __init__(self):
        
        self.x_limits = [-1.8,1.8]
        self.y_limits = [-2.8,2.8]
        self.z_limits = [0.3,2.3]
        
        
        x_limits = self.x_limits
        y_limits = self.y_limits
        z_limits = self.z_limits
        
    def form_L():
        
        positions = []
        pass

        











