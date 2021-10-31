#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 17:02:14 2021

@author: nicole
"""
import utm


class GPS():
        
     
    def __init__(self):
        
        self.sp_utmYX = utm.from_latlon(-22.256714, -45.695668) #lat,lon
        self.pontoFinal_utmXY = [self.sp_utmYX[1],self.sp_utmYX[0]]
        
        
    def Conversao_Geograficas_UTM(self,latitude, longitude):
        
        #sp_geodisicas - utm
        #latitude = y 
        #longitude = x
        
        CoordenadasYX = utm.from_latlon(latitude, longitude)
        CoordenadasXY = [CoordenadasYX[1],CoordenadasYX[0]]
        
        return CoordenadasXY
    
    def get_PontoFinal(self):
        
        return self.pontoFinal_utmXY
    
    def get_teste_PontoFinal(self):
        
        pontoFinal_utmXY = [0,5]
        return pontoFinal_utmXY
    