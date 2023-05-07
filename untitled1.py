#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  7 19:47:16 2023

@author: shreyash
"""
import numpy as np 
import math 

def quaternion_to_euler(x, y, z, w):

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = np.arctan2(t3, t4)

        return  Z

print(quaternion_to_euler(-0.05923353207055521,0.002800071602294586 , 0.9977120145487753,-0.032469744416799486 ))
      




