#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 11:30:08 2021
@author: Mo
Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology
"""


import socket
import time


from mocap import MotionCapture, rosClock

# from mocap import MotionCapture
# from .utils import clamp

class Telloserver:
    """
    Python wrapper to interact with multiple Ryze Tello drones. 
    It is based on the official Tello API for standard Tello drones.
    It uses hardware redundacy (multiple Wi-Fi adapters) to establish unique 
    Wi-Fi connections, allowing for performing swarming smd formation of drones
    that is not officially supported by standard Tello drones.
    
    Tello API's official documentation:
    [1.3](https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf)
    
    """
    ## Global Parameters
    
    # server socket
    TELLO_IP = '192.168.10.1'  # Tello IP address
    CONTROL_UDP_PORT = 8889    # Tello UDP port
    
    # mimic TCP
    MAX_RETRY = 1  # max number to r important commands e.g. land, emergency
    
    def __init__(self, wifi_interface):
        """
    
        Parameters
        ----------
        wifi_interface : string
             wifi interface's name is used to establish unique wifi connections
             between multiple standard Tello drones (as servers with IP:192.168.10.1  
             and Port 8889) and multiple wifi adapters (as clients with IP ranges
             192.168.10.X).
        Returns
        -------
        None.
        """
        self.wifi_interface = wifi_interface
        self.bytes_interface = self.wifi_interface.encode("utf-8")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.bytes_interface)
        
        self.address = (Telloserver.TELLO_IP, Telloserver.CONTROL_UDP_PORT)
    

    
    # =====================  drone's control commands  ====================

    def connect(self):
        """
        Enables sending control commands to a Tello drone
        """
        self.server_socket.sendto('command'.encode(), 0, self.address)
        
    def Disconnect(self):
        """
        closes the UDP channels used for sending commands to a Tello drone
        """
        self.server_socket.close()    
    
    def emergency(self):
        """
        Stops all motors immediately.
        """
        # self.server_socket.sendto('emergency'.encode(), 0, self.address)
        print("Entered Emergency Landing")
        for i in range(Telloserver.MAX_RETRY):
            self.server_socket.sendto('emergency'.encode(), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered   
            
    
    def takeoff(self):
        """
    
        """
        self.server_socket.sendto('takeoff'.encode(), 0, self.address)
        # for i in range(Telloserver.MAX_RETRY):
            # self.server_socket.sendto('takeoff'.encode(), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered   
    

    def land(self): 
        # self.server_socket.sendto('land'.encode(), 0, self.address)
        print("Landing with Land")
        return self.server_socket.sendto('land'.encode(), 0, self.address)

        # for i in range(Telloserver.MAX_RETRY):
           #  self.server_socket.sendto('land'.encode(), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered
        

    def Landing(GT, allDrones):
        
        safe_height = False 
        
        while safe_height == False:
            try:
                print('Received Landing Command')

                altitude = []
                for drone in GT:
                    #altitude = MotionCapture(drone).getPose()[0][2]
                    altitude.append(drone.getPose()[0][2])
                print("Drones are at altitude: ",altitude)
                count = 0

                for drone in allDrones:
                    idx = allDrones.index(drone)
                    if altitude[idx] > 0.35:
                        
                        print("Current Altidute is: ", altitude)            
                        landing_velocity = altitude[idx] * 70 if altitude[idx] < 1 else 70
                        print("Trying to land: Landing Velocity is: ", landing_velocity)
                        drone.cmdVelocity(0,0,-landing_velocity,0)
                        #rosClock.sleepForRate(10)
                    else:
                        drone.emergency()
                        count += 1
                        if count == len(altitude): safe_height = True
                    #rosClock.sleepForRate(10)

            except KeyboardInterrupt:
        
                print('emergency interruption!; aborting all flights ...')
                for drone in allDrones:
                    drone.emergency()
                
                    # rosClock.sleep(.1)
                rosClock.sleepForRate(10)





    def cmdVelocity(self, Vx, Vy, Vz, yawRate): 
        """
        TODO: 
            what is the unit? in cm/sec for vel
            what coordinate sys.? XYZ => END ? in body-frame
        
        
        Sends remote control commands in four channels:
            
        Arguments:
            left_right_velocity: -100~100 (left/right)
            forward_backward_velocity: -100~100 (forward/backward)
            up_down_velocity: -100~100 (up/down)
            yawRate: -100~100 OR yaw ????
            
        """
        def clamp(x: int, min_value: int, max_value: int) -> int:
            return max(min_value, min(max_value, x))
        
        cmd = 'rc {} {} {} {}'.format(
                                    # print(round(Vx)),
                                    clamp(round(Vx),-100,100),
                                    clamp(round(Vy),-100,100),
                                    clamp(round(Vz),-100,100),
                                    clamp(round(yawRate),-100,100))
        # print(cmd)
        self.server_socket.sendto(cmd.encode(), 0, self.address)
        
    
        # =============== receive the drone's onboard data  =================
        
        # TODO 
        
# ===============   Multi-agent API for swarm and formation  ================



class SWARM:
    """
    Multi-agent API for swarm and formation 
    
    """
    
    def __init__(self, wifi_interfaces: list,
                 droneslist:list = None,
                 defaultName = 'Drone'):
        """
        
        Parameters
        ----------
        wifi_interfaces : list
            DESCRIPTION.
            
        droneslist : str = optional
            DESCRIPTION.
        
        defaultName : str, optional
            DESCRIPTION. The default is 'Drone'.
        Returns
        -------
        None.
        """

        if droneslist is None:
            self.droneslist = []
            for i in range(len(wifi_interfaces)):
                self.droneslist.append(defaultName + str(i+1))
        else:
            self.droneslist = droneslist
            
        self.allDrones = []

        for drone in self.droneslist:
            idx = self.droneslist.index(drone)
            drone = Telloserver(wifi_interface=wifi_interfaces[idx])
            self.allDrones.append(drone)
        
        
        self.rosClock = rosClock
 

    def MotionCaptureGroundTruth(self):
        self.GroundTruth = []
        for drone in self.droneslist:
            self.GroundTruth.append(MotionCapture(drone))  
        return self.GroundTruth


