#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 17:17:26 2021

@author: nicole
"""

class Filtros():
    
    def __init__(self):
        self.somaPontoMedio = 0
        self.iPontoMedio = 0
        self.mediaPontoMedio = 0
        
        self.somaMediaMovel = 0
        self.iMediaMovel = 0
        self.mediaMediaMovel = 0
        
    def Filtro_IIR(self, constant, vetorXY_anterior, vetorXY_atual):
        #IIR Infinite Impulse Response (Filtro recursivo)
        
        filter_constant = constant
        
        vetorXY_atual[0] = round(filter_constant * vetorXY_anterior[0] + (1 - filter_constant) * vetorXY_atual[0],2) #Latitude - y
        vetorXY_atual[1] = round(filter_constant * vetorXY_anterior[1] + (1 - filter_constant) * vetorXY_atual[1],2) #Longitude - x
            
        return vetorXY_atual

    # inicio, numero de loop, soma, variavel_x
    def Filtro_MediaMovel(self, n, x): 
    # Passa a cada leitura até atingir o valor de n.
    
        #Media Movel        
        if self.iMediaMovel <= n:
            self.somaMediaMovel = self.somaMediaMovel + x 
            self.iMediaMovel+=1
            
            return self.mediaMediaMovel
            
        else:                      
            self.mediaMediaMovel = self.somaMediaMovel/self.iMediaMovel
            self.iMediaMovel = 0
            self.somaMediaMovel = 0
            return self.mediaMediaMovel
        
    def Filtro_PontoMedio(self, n, vetor_amostras): 
    # Leitura de um vetor 
        
        tamanho_vetor = n
        
        while n >= 0:            
            self.somaPontoMedio = self.somaPontoMedio + vetor_amostras[n]
            n -= 1
        self.mediaMediaMovel = self.somaPontoMedio/tamanho_vetor
        
        return self.mediaMediaMovel
            
            
            
            
            
            
            
            
            
            
            
            
        