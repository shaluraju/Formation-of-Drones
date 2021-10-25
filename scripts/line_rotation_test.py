# -*- coding: utf-8 -*-
"""
Created on Wed Oct 20 10:20:41 2021

@author: sssha
"""
from Shape_vectors import LineFormation

initial_pos = [[0,-0.5,0.8],[0,0,0.8],[0,0.5,0.8]]
theta = 1

while theta < 64:

    l_goal = LineFormation.d_rotation(initial_pos, 0.5, theta)
    print("Leader Location: ", l_goal)
            
    error = LineFormation.form(0.5,0.5,0.0,[l_goal,initial_pos[1],initial_pos[2]] )
    print("error: ",error)
    goal = []
    for i in range(len(error)):
        x = initial_pos[i+1][0] + error[i][0]
        y = initial_pos[i+1][1] + error[i][1]
        z = initial_pos[i+1][2] + error[i][2]
        goal.append([x,y,z])
    print("goal: ", goal)
    
    initial_pos = [l_goal, goal[0], goal[1]]
    
    theta +=1
    
