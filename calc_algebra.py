#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 19:35:55 2021

@author: nicole
"""
import numpy as np

class Calc_Algebra():
    
    
    def get_Modulo(self, X,Y):
        
        modulo = np.sqrt(np.square(X) + np.square(Y))
        
        return modulo
    
    def get_Vetor(self, A,B):
        
        vetor = np.array([B[0] - A[0], B[1] - A[1]]) #vetorReferencia[x,y]
        
        return vetor
    
    def get_AnguloVetores_rad(A, B, mod_a, mod_b):
        
        angulo = np.arccos(((A[0] * B[0]) + (A[1] * B[1]))/(mod_a * mod_b))
        
        return angulo
    
    def get_AnguloVetores_graus(self,A, B, C): #(A,B),(C,D)pontos
        
        
        vetorAB = self.get_Vetor(A, B)
        vetorBC = self.get_Vetor(B, C)
        
        mod_AB = self.get_Modulo(vetorAB[0], vetorAB[1])
        mod_C = self.get_Modulo(vetorBC[0], vetorBC[1])
        
        
        vet = (vetorAB[0] * vetorBC[0]) + (vetorAB[1] * vetorBC[1])
        mod = (mod_AB * mod_C)
        
        
        angulo = np.arccos(vet/mod)
        
        
        angulo_g = (angulo*180)/np.pi
        
        return angulo_g
    
    def get_AnguloVetores_Unitario_graus(self,A, B, angulo): #(A,B),(C,D)pontos
        
        vetorAB = self.get_Vetor(A, B)
        vetorC = self.get_VetorUnitario(angulo)
       
        
        mod_AB = self.get_Modulo(vetorAB[0], vetorAB[1])
        mod_C = self.get_Modulo(vetorC[0], vetorC[1])
        
        angulo = np.arccos(((vetorAB[0] * vetorC[0]) + (vetorAB[1] * vetorC[1]))/(mod_AB * mod_C))
        angulo_g = (angulo*180)/np.pi
        
        return angulo_g
    
    def get_CoeficienteAngular(A, B):
        
        DeltaY = (B[1] - A[1])
        DeltaX = (B[0] - A[0])
    
        Coef_m = DeltaY/DeltaX
        Coef_m_rad = np.arctan(Coef_m)
        
        #conversao Rad para Angulo
        Coef_m_angulo = (Coef_m_rad*180)/np.pi
        
        return Coef_m_angulo
    
    def get_VetorUnitario(self,sensor):
        
        vetorUnitario = [np.cos(sensor), np.sin(sensor)]
        
        return vetorUnitario
    
    def get_ProdutoVetorial(self,pontoA, pontoB, pontoC):
        
        vetorAB = self.get_Vetor(pontoA, pontoB)
        vetorBC = self.get_Vetor(pontoB, pontoC)
        
        vetorZ = (vetorAB[1]*vetorBC[0]) - (vetorAB[0]*vetorBC[1])
        
        return vetorZ
    
    def teste_Modulo(self):     
        
        x = 3
        y = 2
        
        modulo = self.get_Modulo(x,y)
        print('teste_Modulo: ', modulo)
        
    def teste_AnguloVetores_graus(self):     
        
        A = [5277007.66,299828.85]
        B = [6860109.97,389777.51]
        C = [7538593.13368571,428321.255205525]
        
        angulo = self.get_AnguloVetores_graus(A, B, C)
        print('angulo: ', angulo)
        
        
        
        