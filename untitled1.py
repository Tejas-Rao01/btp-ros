#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 12 17:55:45 2023

@author: shreyash
"""
import numpy as np 


_, SL, SR = 0.120000000000001 ,-0.4369999999999976, -0.1836000000000002

delta_x, delta_y, delta_theta = 0.06600000000000072, -0.02400000000000091,0.006119999999999237
robotTheta = np.a


theta_not = delta_theta / 2 + -2.292934287564282 + np.pi/2


a = (delta_x+ delta_y ) * 2 / (np.cos(theta_not) + np.sin(theta_not))
b = delta_theta * 0.4
sr = (a + b) / 2 
sl = (a - b) / 2
print(sr)
print(sl)
print(a)
print(b)