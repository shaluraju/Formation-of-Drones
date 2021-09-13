#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 15:09:09 2021

@author: Mo

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

# from TelloServer import  Telloserver
from TelloServer import  *

import time

drone1 = Telloserver(wifi_interface="wlx9cefd5faea28")

drone2 = Telloserver(wifi_interface="wlx9cefd5fb4bdd")


drone1.connect()
drone2.connect()

drone1.takeoff()
drone2.takeoff()

time.sleep(1)

drone1.land()
drone2.land()