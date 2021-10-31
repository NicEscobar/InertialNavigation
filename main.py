#python3 main.py --device=/dev/ttyACM1 --baudrate=115200
#https://www.youtube.com/watch?v=UGjjP45wrKQ

from filtros import Filtros
#from sensores import Sensores
from gps import GPS
from calc_algebra import Calc_Algebra

import time
#import RPi.GPIO as gpio
import threading
#import sys, os
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib import animation

import pandas as pd
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

'''
# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil

gpio.setmode(gpio.BCM)

gpio.setup(18,gpio.OUT)
gpio.setup(12,gpio.OUT)

pwm = gpio.PWM(18, 50)
motor = gpio.PWM(12, 50)

gpio.output(18,gpio.LOW)
gpio.output(12,gpio.LOW)

pwm.start(0)
motor.start(0)

pwm.ChangeDutyCycle(6.6)
motor.ChangeDutyCycle(7.2)
time.sleep(2)
'''
#obj_sensor = Sensores();
obj_filtros = Filtros();
obj_gps = GPS();
obj_calc_algebra = Calc_Algebra();

anguloGiro = 0
gps_xy_anterior = [0,0]
gps_xy_atual = [3,3]
gps_xy_final = [5,1]
#gps_xy_final = obj_gps.get_PontoFinal()
msg_type = "GLOBAL_POSITION_INT"
msg = True

n_satelites = 0

sair = False

df = pd.DataFrame({
                    'gps_x_anterior': [0],
                    'gps_y_anterior': [0],
                    'gps_x_atual': [0],
                    'gps_y_atual': [0],
                    'gps_x_final': [0],
                    'gps_y_final': [0],
                    'angulo(erro)': [0],
                    'sinal(erro)': [0],
                    'corr': [0],
                    'servo': [0],
                    
                })

kp = 5 #proporcional. O quanto agrecivo o carro vai virar. Muda o servo. Menor: varia mais suave
kd = 1 #derivativo. Deixa o proporcional mais rápido quando o derivativo por grande
ki = 0 #integrativo
erro = 0
erro_ant = 0
sinalGiro = 0
der_erro = 0
corr = 0

fig = plt.Figure(figsize=(5,4), dpi=100)
ax = fig.add_subplot(111)
#---------------------------Threads
def thread_MPU():
    
    global anguloGiro
    
    try:
        anguloGiro = obj_sensor.get_angulos_MPU6050();
    except:
        anguloGiro = 0
    
def thread_Principal():
    
    global sair, gerarCSV
    global anguloGiro, gps_xy_anterior, gps_xy_atual, gps_xy_final, msg, msg_type, df, n_satelites
    global kp, kd, ki, erro, erro_ant, sinalGiro,der_erro,corr       

    i = 0
    # create a mavlink serial instance
    #master = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
    
    # wait for the heartbeat msg to find the system ID
    #master.wait_heartbeat()

    # request data to be sent at the given rate
    #master.mav.request_data_stream_send(master.target_system, master.target_component, 
    #mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    
    #gps_xy_final = obj_gps.get_PontoFinal()
      
    while sair == False:
        
        # grab a mavlink message
        #msg = master.recv_match(blocking=True)
        
        if msg:       
            # handle the message based on its type
            #msg_type = msg.get_type()
             
            
            if msg_type == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            elif msg_type == "RC_CHANNELS_RAW": 
                #handle_rc_raw(msg)
                pass
            elif msg_type == "HEARTBEAT":
                #handle_heartbeat(msg)
                #print('heart',msg)
                pass
            elif msg_type == "GPS_RAW_INT":
                n_satelites = msg.satellites_visible
                pass
            
            elif msg_type == "GLOBAL_POSITION_INT":
               
                '''
                lat = float(msg.lat)/10000000
                lon = float(msg.lon)/10000000
                
                gps_xy_atual_bruto = obj_gps.Conversao_Geograficas_UTM(lat, lon)
                             
                gps_xy_atual = obj_filtros.Filtro_IIR(0.3, gps_xy_anterior, gps_xy_atual_bruto)                     
                '''
                
                
                if(gps_xy_anterior != gps_xy_atual):
                    erro = obj_calc_algebra.get_AnguloVetores_Unitario_graus(gps_xy_atual, gps_xy_final, anguloGiro)
                    #erro = obj_calc_algebra.get_AnguloVetores_graus(gps_xy_anterior, gps_xy_atual, gps_xy_final)
                    sinalGiro = obj_calc_algebra.get_ProdutoVetorial(gps_xy_anterior, gps_xy_atual, gps_xy_final)
                
                
                if erro > 180:
                    erro = erro - 360
                elif erro < -180:
                    erro = erro + 360
                
                
                der_erro = erro - erro_ant
                               
                corr = erro*kp + der_erro*kd
                
                if sinalGiro > 0:
                    corr = corr * (-1)
                
                #para não ultrapassar o valor de 2000 a 1000 do servo
                if corr > 500:
                    corr = 500
                elif corr < -500:
                    corr = -500
        
                servo = 1500.0 + corr
                servo = servo - 1000
                servo = 5.0*servo/1000
                servo = servo + 4.5
               
                
                #pwm.ChangeDutyCycle(servo)
                                       
                #print("anguloGiro: {:.2f} sinal: {:.2f} erro: {:.2f} corr: {:.2f} servo: {:.2f} der_erro: {:.2f}"
                #.format(anguloGiro, sinalGiro, erro, corr, servo, der_erro))
                #print("x_anterior: {:.2f} y_anterior: {:.2f} x_atual: {:.2f} y_atual: {:.2f} x_sp: {:.2f} y_: {:.2f}"
                #.format(gps_xy_anterior[0], gps_xy_anterior[1],gps_xy_atual[0],gps_xy_atual[1],gps_xy_final[0],gps_xy_final[1]))
               
                erro_ant = erro
                
                #Adicionando valores no dataframe
                novaEntrada_df = [gps_xy_anterior[0],
                                  gps_xy_anterior[1],
                                  gps_xy_atual[0],
                                  gps_xy_atual[1],
                                  gps_xy_final[0],
                                  gps_xy_final[1],
                                  erro,
                                  sinalGiro,
                                  corr,
                                  servo]                         
                
                df.loc[len(df)] = novaEntrada_df
                
                '''
                if(gps_xy_final != gps_xy_atual):
                    motor.ChangeDutyCycle(7.6)
                else:
                    motor.ChangeDutyCycle(6)
                '''
      
               
    print('Gerou loog')
    GerarCSV(df)           
                
               
#-------------------------------------------Main--------------------------  
def animate(i):

    global anguloGiro, gps_xy_anterior,gps_xy_atual,gps_xy_final
    
    ax.clear()
    
    x = [gps_xy_anterior[0],gps_xy_atual[0],gps_xy_final[0]]
    y = [gps_xy_anterior[1],gps_xy_atual[1],gps_xy_final[1]]
    
    x2 = [gps_xy_atual[0],gps_xy_final[0]]
    y2 = [gps_xy_atual[1],gps_xy_final[1]]
    
    ax.plot(x, y, color='blue')
    ax.plot(x2, y2, color='orange')
      


def GerarCSV(df):
    
    df.to_csv('log.csv')
    
    
def Sair():
    
    global sair
    
    sair = True
    
try:
    #threading.Thread(target=thread_MPU).start()
    thread = threading.Thread(target=thread_Principal).start()
       
    tk = Tk()
    tk.title('Navegação')
    tk.geometry("1200x1200")
    
    botaoSair = Button(tk, text="Parar > Gerar Log", command=Sair)
    botaoSair.grid(column=1,row=9,padx=10,pady=10)
    
    #botaoGrafico = Button(tk, text="Grafico", command=GerarGrafico)
    #botaoGrafico.grid(column=2,row=9,padx=10,pady=10)
    
    titulo_GPS = Label(tk, text='Coordenadas GSP em UTM ')
    titulo_GPS.grid(column=1,row=1,padx=10,pady=10)
    
    texto_GPS_Anterior = Label(tk, text='GPS Ponto Anterior [x,y]: ')
    texto_GPS_Anterior.grid(column=1,row=2,padx=10,pady=10)
    GPS_Anterior = Label(tk, text=gps_xy_anterior)
    GPS_Anterior.grid(column=1,row=3)
    
    texto_GPS_Atual = Label(tk, text='GPS Ponto Atual [x,y]: ')
    texto_GPS_Atual.grid(column=2,row=2,padx=10,pady=10)
    GPS_Atual = Label(tk, text=gps_xy_atual)
    GPS_Atual.grid(column=2,row=3)
    
    texto_GPS_Final = Label(tk, text='GPS Ponto Final [x,y]: ')
    texto_GPS_Final.grid(column=3,row=2,padx=10,pady=10)
    GPS_Final = Label(tk, text=gps_xy_final)
    GPS_Final.grid(column=3,row=3)
    
    texto_GPS_Satelites = Label(tk, text='Satélites disponíveis: ')
    texto_GPS_Satelites.grid(column=4,row=2,padx=10,pady=10)
    GPS_Satelites = Label(tk, text=n_satelites)
    GPS_Satelites.grid(column=4,row=3)
    
    titulo_PD = Label(tk, text='Proporcial Derivativo - PD')
    titulo_PD.grid(column=1,row=4,padx=10,pady=10)
    
    texto_correcaoErro = Label(tk, text='Angulo entre Vetores (Erro): ')
    texto_correcaoErro.grid(column=1,row=5,padx=10,pady=10)
    correcaoErro = Label(tk, text=erro)
    correcaoErro.grid(column=1,row=6)
    
    texto_correcaoGiro = Label(tk, text='Sinal de Giro (dir/esq = +/-): ')
    texto_correcaoGiro.grid(column=2,row=5,padx=10,pady=10)
    correcaoGiro = Label(tk, text=erro)
    correcaoGiro.grid(column=2,row=6)
    
    texto_corrErro = Label(tk, text='Correção do Erro (Corr): ')
    texto_corrErro.grid(column=3,row=5,padx=10,pady=10)
    corrErro = Label(tk, text=corr)
    corrErro.grid(column=3,row=6)
    
    texto_correcaoErroAnterior = Label(tk, text='Correção em Angulo (Erro Anterior): ')
    texto_correcaoErroAnterior.grid(column=4,row=5,padx=10,pady=10)
    correcaoErro = Label(tk, text=erro_ant)
    correcaoErro.grid(column=4,row=6)
    
    texto_derErro = Label(tk, text='Derivada do Erro: ')
    texto_derErro.grid(column=5,row=5,padx=10,pady=10)
    correcaoErro = Label(tk, text=der_erro)
    correcaoErro.grid(column=5,row=6)
    
    '''
    x = [gps_xy_anterior[0],gps_xy_atual[0],gps_xy_final[0]]
    y = [gps_xy_anterior[1],gps_xy_atual[1],gps_xy_final[1]]
    
    x2 = [gps_xy_atual[0],gps_xy_final[0]]
    y2 = [gps_xy_atual[1],gps_xy_final[1]]
    
    fig = plt.Figure(figsize=(5,4), dpi=100)
    ax = fig.add_subplot(111)
    ax.plot(x, y, color='tab:blue')
    
    canvas = FigureCanvasTkAgg(fig, master=tk)
    canvas.get_tk_widget().grid(column=2,row=8,columnspan=4, rowspan=11,padx=10,pady=10)   
    '''
    canvas = FigureCanvasTkAgg(fig, master=tk)
    canvas.get_tk_widget().grid(column=2,row=8,columnspan=4, rowspan=11,padx=10,pady=10)
    
    ani = animation.FuncAnimation(fig, animate, interval=1000)
   
    tk.mainloop()
    
      
except(KeyboardInterrupt, SystemExit):
    #gpio.cleanup()
    
    thread.__stop()
    thread.join()
    time.sleep(5)
    pass
