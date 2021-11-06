#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  1 11:44:20 2021

@author: nicole
"""


class IntefaceGrafica(tk.Frame):
    
    def __init__(self, master=None):
        
        tk.__init__(self, master)        
        
        tk.title('Navegação')
        tk.geometry("1200x600")
        tk.configure(bg='#FFF')
        
        titulo_GPS = Label(tk, text='Coordenadas GSP em UTM',bg='#FFF',font='Helvetica 12 bold')
        titulo_GPS.grid(column=1,row=1,padx=10,pady=10)
        
        texto_GPS_Anterior = Label(tk, text='GPS Ponto Anterior [x,y]: ',bg='#d5f5e3',padx=5,pady=5,font='Helvetica 10 bold')
        texto_GPS_Anterior.grid(column=1,row=2,padx=10,pady=10)
        GPS_Anteriorx = Label(tk, text=gps_xy_anterior[0],bg='#e8f8f5',padx=5,pady=5)
        GPS_Anteriorx.grid(column=1,row=3)
        GPS_Anteriory = Label(tk, text=gps_xy_anterior[1],bg='#e8f8f5',padx=5,pady=5)
        GPS_Anteriory.grid(column=1,row=4)
        
        texto_GPS_Atual = Label(tk, text='GPS Ponto Atual [x,y]: ',bg='#fcf3cf',padx=5,pady=5,font='Helvetica 10 bold')
        texto_GPS_Atual.grid(column=2,row=2,padx=10,pady=10)
        GPS_Atual = Label(tk, text=gps_xy_atual[0],bg='#fef9e7',padx=5,pady=5)
        GPS_Atual.grid(column=2,row=3)
        GPS_Atual = Label(tk, text=gps_xy_atual[1],bg='#fef9e7',padx=5,pady=5)
        GPS_Atual.grid(column=2,row=4)
        
        texto_GPS_Final = Label(tk, text='GPS Ponto Final [x,y]: ',bg='#fae5d3',padx=5,pady=5,font='Helvetica 10 bold')
        texto_GPS_Final.grid(column=3,row=2,padx=10,pady=10)
        GPS_Final = Label(tk, text=gps_xy_final[0],bg='#fdf2e9')
        GPS_Final.grid(column=3,row=3)
        GPS_Final = Label(tk, text=gps_xy_final[1],bg='#fdf2e9')
        GPS_Final.grid(column=3,row=4)
        
        texto_GPS_Satelites = Label(tk, text='Satélites disponíveis: ',bg='#f8f9f9',padx=5,pady=5,font='Helvetica 10 bold')
        texto_GPS_Satelites.grid(column=4,row=2,padx=10,pady=10)
        GPS_Satelites = Label(tk, text=n_satelites,bg='#FFF')
        GPS_Satelites.grid(column=4,row=3)
        
        titulo_PD = Label(tk, text='Proporcial Derivativo - PD',bg='#FFF',font='Helvetica 12 bold')
        titulo_PD.grid(column=1,row=5,padx=10,pady=10)
        
        texto_correcaoErro = Label(tk, text='Angulo entre Vetores (Erro): ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
        texto_correcaoErro.grid(column=1,row=6,padx=10,pady=10)
        correcaoErro = Label(tk, text=erro,bg='#FFF')
        correcaoErro.grid(column=1,row=7)
        
        texto_correcaoGiro = Label(tk, text='Sinal de Giro (dir/esq = +/-): ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
        texto_correcaoGiro.grid(column=2,row=6,padx=10,pady=10)
        correcaoGiro = Label(tk, text=erro,bg='#FFF')
        correcaoGiro.grid(column=2,row=7)
        
        texto_corrErro = Label(tk, text='Correção do Erro (Corr): ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
        texto_corrErro.grid(column=3,row=6,padx=10,pady=10)
        corrErro = Label(tk, text=corr,bg='#FFF')
        corrErro.grid(column=3,row=7)
        
        texto_correcaoErroAnterior = Label(tk, text='Correção em Angulo (Erro Anterior): ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
        texto_correcaoErroAnterior.grid(column=4,row=6,padx=10,pady=10)
        correcaoErro = Label(tk, text=erro_ant,bg='#FFF')
        correcaoErro.grid(column=4,row=7)
        
        texto_derErro = Label(tk, text='Derivada do Erro: ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
        texto_derErro.grid(column=5,row=6,padx=10,pady=10)
        correcaoErro = Label(tk, text=der_erro,bg='#FFF')
        correcaoErro.grid(column=5,row=7)
        
        texto_GPS_Final_input = Label(tk, text='Final (Latitude): ',bg='#e59866',fg="#fff",padx=5,pady=5,font='Helvetica 10 bold')
        texto_GPS_Final_input.grid(column=1,row=8,padx=10,pady=10)
        GPS_Final_input_lat = Entry(tk)
        GPS_Final_input_lat.grid(column=1,row=9)
        
        texto_GPS_Final_input = Label(tk, text='Final (Longitude): ',bg='#e59866',fg="#fff",padx=5,pady=5,font='Helvetica 10 bold')
        texto_GPS_Final_input.grid(column=2,row=8,padx=10,pady=10)
        GPS_Final_input_lon = Entry(tk)
        GPS_Final_input_lon.grid(column=2,row=9)
        
        botaoPonto = Button(tk, text="Novo Ponto Final", command=SalvarNovoPontoFinal,bg='#dc7633',fg="#fff")
        botaoPonto.grid(column=3,row=9,padx=10,pady=10)
        
        botaoSair = Button(tk, text="Gráfico", command=GerarCSV,bg='#7fb3d5',fg="#fff")
        botaoSair.grid(column=4,row=9,padx=10,pady=10)
        
        botaoGrafico = Button(tk, text="Parar Motor", command=PararMotor,bg='#f1948a',fg="#fff")
        botaoGrafico.grid(column=4,row=10,padx=10,pady=10)
        
        botaoSair = Button(tk, text="SAIR", command=Sair,bg='#c0392b',fg="#fff")
        botaoSair.grid(column=4,row=11,padx=10,pady=10)
        
        '''
        canvas = FigureCanvasTkAgg(fig, master=tk)
        canvas.get_tk_widget().grid(column=2,row=10,columnspan=4, rowspan=11,padx=10,pady=10)
        
        ani = animation.FuncAnimation(fig, animate, interval=20)
       '''