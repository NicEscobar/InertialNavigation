#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 19:05:23 2021

@author: nicole
"""
import serial

class Sensores():
    
    def __init__(self):
        
        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200) 
     
    def get_angulos_MPU6050(self):
        
        data = self.arduino.readline().decode('utf-8')
        
        
        if data != []:           
            
            try:
                split_value = data.split()              
                # Convert string data to float
                s = list(map(lambda x: float(x), split_value))                  
                return s[0]  
            
            except:             
                return 0               
        else:        
            return 0

    
    