#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 17:02:14 2021

@author: nicole
"""
import utm


class GPS():
        

    def __init__(self):        
        self.sp_utmYX = utm.from_latlon(-22.25817, -45.69578) #lat,lon
        self.pontoFinal_utmXY = [self.sp_utmYX[1],self.sp_utmYX[0]]
        
        
    def Conversao_Geograficas_UTM(self,latitude, longitude):
        
        #sp_geodisicas - utm
        #latitude = y 
        #longitude = x
        
        CoordenadasYX = utm.from_latlon(latitude, longitude)
        
        CoordenadasXY_m = [CoordenadasYX[1],CoordenadasYX[0]]
        
        CoordenadasXY_km = [CoordenadasXY_m[0], CoordenadasXY_m[1]]
        
        return CoordenadasXY_km
    
    def get_PontoFinal(self):
        
        return self.pontoFinal_utmXY
    
    def get_FaixaErroPontoFinal_PN(self, Pfinal):
        
        xP = Pfinal[0]*1.000000002
        yP = Pfinal[1]*1.000000002
        
        xN = Pfinal[0]*0.000000098
        yN = Pfinal[1]*0.000000098
        
        max = [xP,yP]
        min = [xN, yN]
        
        vet = [max,min]
        
        return vet
    
    def set_PontoFinal(self, lat, lon):
        
        coord = self.Conversao_Geograficas_UTM(lat, lon)
       
        self.pontoFinal_utmXY = [coord[1], coord[0]]
            
    def get_teste_PontoFinal(self):
        
        pontoFinal_utmXY = [0,5]
        return pontoFinal_utmXY
    