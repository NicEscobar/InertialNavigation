#python3 main.py --device=/dev/ttyACM1 --baudrate=115200
#https://www.youtube.com/watch?v=UGjjP45wrKQ

from filtros import Filtros
from sensores import Sensores
from gps import GPS
from calc_algebra import Calc_Algebra

import time
import RPi.GPIO as gpio
import threading
import sys, os
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib import animation

import pandas as pd
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil


gpio.setmode(gpio.BCM)

gpio.setup(18,gpio.OUT)
gpio.setup(25,gpio.OUT)

pwm = gpio.PWM(18, 50)
motor = gpio.PWM(25, 50)

gpio.output(18,gpio.LOW)
gpio.output(25,gpio.LOW)

pwm.start(0)
motor.start(0)

pwm.ChangeDutyCycle(6.6)
motor.ChangeDutyCycle(7.6)
time.sleep(2)

obj_sensor = Sensores();
obj_filtros = Filtros();
obj_gps = GPS();
obj_calc_algebra = Calc_Algebra();

anguloGiro = 0
 
gps_xy_anterior = obj_gps.Conversao_Geograficas_UTM(-22.25815,-45.69586)
gps_xy_atual = obj_gps.Conversao_Geograficas_UTM(-22.25819,-45.69580)
gps_xy_final = obj_gps.get_PontoFinal()



n_satelites = 0

sair = False
pausar = False

df = pd.DataFrame({
                    'gps_x_anterior': [],
                    'gps_y_anterior': [],
                    'gps_x_atual': [],
                    'gps_y_atual': [],
                    'gps_x_final': [],
                    'gps_y_final': [],
                    'angulo(erro)': 0,
                    'sinal(erro)': 0,
                    'corr': 0,
                    'servo': 0,
                    'gps_x_atual_real':[],
                    'gps_y_atual_real':[]                   
                })


kp = 4 #proporcional. O quanto agrecivo o carro vai virar. Muda o servo. Menor: varia mais suave
kd = 0.5 #derivativo. Deixa o proporcional mais rápido quando o derivativo por grande
ki = 0 #integrativo

erro = 0
erro_ant = 0
sinalGiro = 0
der_erro = 0
corr = 0


motorComand = True

fig = plt.Figure(figsize=(5,4), dpi=100)
ax = fig.add_subplot(111)

#---------------------------Threads

       
       
    
def thread_MPU():
    
    global anguloGiro, sair
    
    while sair == False:
        try:
            anguloGiro = obj_sensor.get_angulos_MPU6050();
        except:
            anguloGiro = 0
    
def thread_Principal():
    
    global sair
    global anguloGiro, gps_xy_anterior, gps_xy_atual, gps_xy_final, n_satelites
    global kp, kd, ki, erro, erro_ant, sinalGiro,der_erro,corr,depoisDasPrimeirasAmostras       

    contGPS = 0
    depoisDasPrimeirasAmostras = 0
    erroVetores = 0
    anguloFinal = 0

    
    # create a mavlink serial instance
    master = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
    
    # wait for the heartbeat msg to find the system ID
    master.wait_heartbeat()

    # request data to be sent at the given rate
    master.mav.request_data_stream_send(master.target_system, master.target_component, 
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
      
    while sair == False:

        while pausar == False:        
            # grab a mavlink message
            msg = master.recv_match(blocking=True)
            
            if msg:       
                # handle the message based on its type
                msg_type = msg.get_type()
                 
                
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
                   
                                           
                    lat = float(msg.lat)/10000000
                    lon = float(msg.lon)/10000000
                    
                    gps_xy_atual_bruto = obj_gps.Conversao_Geograficas_UTM(lat, lon)
                                 
                    gps_xy_atual_real = obj_filtros.Filtro_IIR(0.3, gps_xy_anterior, gps_xy_atual_bruto)                                                            
                    
                    sinalGiro = obj_calc_algebra.get_ProdutoVetorial(gps_xy_anterior, gps_xy_atual, gps_xy_final)
                    
                    #Pega a faixa de tolerancia do ponto final
                    gps_xy_final_Faixa = obj_gps.get_FaixaErroPontoFinal_PN(gps_xy_atual)
                      
                    if gps_xy_atual_real < gps_xy_atual and gps_xy_atual_real > gps_xy_atual:               
                        erroVetores = obj_calc_algebra.get_AnguloVetores_graus(gps_xy_anterior, gps_xy_atual, gps_xy_final)
                    
                    if(sinalGiro > 0):
                        erroVetores = erroVetores*(-1)
                        
                    erro = anguloGiro - erroVetores
                    
                    if erro > 180:
                        erro = erro - 360
                    elif erro < -180:
                        erro = erro + 360
                                  
                    der_erro = erro - erro_ant
                                   
                    corr = erro*kp + der_erro*kd
                    
                    if sinalGiro < 0:
                        corr = corr * (-1)
                    
                    #para não ultrapassar o valor de 2000 a 1000 do servo
                    if corr > 500:
                        corr = 500
                    elif corr < -500:
                        corr = -500
            
                    servo = 1500.0 + corr
                    servo = servo - 1000
                    servo = 5.0*servo/1000
                    servo = servo + 5
                                   
                    ControleServo(servo)
                    motor.ChangeDutyCycle(7.8)
                    
                    print("anguloGiro: {:.2f} sinal: {:.2f} erro: {:.2f} servo: {:.2f}  GPS_ATUAL_x: {:.2f} GPS_ATUAL_y: {:.2f}"
                    .format(anguloGiro, sinalGiro, erro, servo, gps_xy_atual_real[0], gps_xy_atual_real[1]))
                  
                    
                    #print('ponto final', gps_xy_final, 'ponto atual', gps_xy_atual, 'ponto_anterior', gps_xy_anterior)
                    #Se chegar perto do ponto final, para o motor
                    
                    
                    #Adiciona novos valores no dataframe
                    Adicionar_Dataframe(gps_xy_anterior,gps_xy_atual,gps_xy_final,erro,sinalGiro,corr,servo,gps_xy_atual_real)
                                    
                    #time.sleep(0.2)
                    
                    erro_ant = erro                                                                      
                    contGPS -= 1
               
#------------------------------------sinal-------Main--------------------------  
def animate(i):

    global anguloGiro, gps_xy_anterior,gps_xy_atual,gps_xy_final
    
    ax.clear()
    
    x = [gps_xy_anterior[0],gps_xy_atual[0],gps_xy_final[0]]
    y = [gps_xy_anterior[1],gps_xy_atual[1],gps_xy_final[1]]
    
    x2 = [gps_xy_atual[0],gps_xy_final[0]]
    y2 = [gps_xy_atual[1],gps_xy_final[1]]
    
    ax.plot(x, y, color='blue')
    ax.plot(x2, y2, color='orange')
      
def SalvarNovoPontoFinal():
    
    global gps_xy_final
    
    campo_lat = float(GPS_Final_input_lat.get())
    campo_lon = float(GPS_Final_input_lon.get())
    
    if(campo_lat!='' and campo_lon!=''):
        obj_gps.set_PontoFinal(campo_lat,campo_lon)
        
        l1.config(text=gps_xy_final)
    else:
        print('Preencha os campos lat e lon')
   
def Adicionar_Dataframe(gps_xy_anterior,gps_xy_atual,gps_xy_final,erro,sinalGiro,corr,servo,gps_xy_atual_2):
    
    global df
    
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
                      servo,
                      gps_xy_atual_2[0],
                      gps_xy_atual_2[1]]                         
    
    df.loc[len(df)] = novaEntrada_df
    
    
def GerarCSV():
    
    global df
    
    
    ax = plt.gca()
     
    if df.index.isin([('0')]).any():
        df = df.drop(0)
       
    '''
    #Rota do ponto anterior
    df.plot(kind='line',x='gps_x_anterior',y='gps_y_anterior',color='red',ax=ax,marker="o")
    df.plot(kind='line',x='gps_x_final',y='gps_y_final',color='magenta',ax=ax,marker="o")
    '''
    #Reta primeiro ponto ao final
    #df.plot(kind='scatter',x='gps_x_atual',y='gps_y_atual',color='blue',ax=ax,marker="o")
    
    #Pontos do GPS 
    df.plot(kind='scatter',x='gps_x_atual_real',y='gps_y_atual_real',color='blue',ax=ax,marker="o")
    df.plot(kind='line',x='gps_x_final',y='gps_y_final',color='magenta',ax=ax,marker="o")
    plt.show()
    
    #plt.xticks(values,x)
    

def Pausar():
    
    global pausar
    
    
    
    if pausar == False:
        pausar = True
        print('Parado')
    else:
        pausar = False
        print('Rodando')
   
   
def Sair():
    
    global sair,tk, pausar, df
    
    file = 'log_GPS_Fixo.csv'
    print('Pronto para finalizar')
    
    #Desligo o motor
    pausar = True
    
    time.sleep(1)
    
    #Sai das threads
    sair = True
    
    df.to_csv(file)
    
    #Destroi a interface
    tk.destroy()

def ControleServo(servo):
    global pwm
    
    pwm.ChangeDutyCycle(servo)
    
def ControleMotor(comando):
    
 
    
    #COMANDO:
    #0 = PARAR
    #1 = LIGAR
    
    if comando == 0:
       
        print('Motor parado')
    elif comando == 1:
       
        print('Motor ligado')
        
        
def PararMotor():
    
   
    print('Motor parado')  
    motorComand = False
    
try:
    
    thread1 = threading.Thread(target=thread_MPU).start()
    thread2 = threading.Thread(target=thread_Principal).start()
    #thread3 = threading.Thread(target=thread_MOTOR).start() 
     
    
    tk = Tk()
    
    tk.title('Navegação')
    tk.geometry("700x600")
    tk.configure(bg='#FFF')
    
    titulo_GPS = Label(tk, text='Coordenadas GSP em UTM',bg='#FFF',font='Helvetica 12 bold')
    titulo_GPS.grid(column=1,row=1,padx=10,pady=10)
    
    texto_GPS_Anterior = Label(tk, text='GPS Ponto 1 [x,y]: ',bg='#d5f5e3',padx=5,pady=5,font='Helvetica 10 bold')
    texto_GPS_Anterior.grid(column=1,row=2,padx=10,pady=10)
    GPS_Anteriorx = Label(tk, text=gps_xy_anterior[0],bg='#e8f8f5',padx=5,pady=5)
    GPS_Anteriorx.grid(column=1,row=3)
    GPS_Anteriory = Label(tk, text=gps_xy_anterior[1],bg='#e8f8f5',padx=5,pady=5)
    GPS_Anteriory.grid(column=1,row=4)
    
    texto_GPS_Atual = Label(tk, text='GPS Ponto 2 [x,y]: ',bg='#fcf3cf',padx=5,pady=5,font='Helvetica 10 bold')
    texto_GPS_Atual.grid(column=2,row=2,padx=10,pady=10)
    GPS_Atual = Label(tk, text=gps_xy_atual[0],bg='#fef9e7',padx=5,pady=5)
    GPS_Atual.grid(column=2,row=3)
    GPS_Atual = Label(tk, text=gps_xy_atual[1],bg='#fef9e7',padx=5,pady=5)
    GPS_Atual.grid(column=2,row=4)
    
    texto_GPS_Final = Label(tk, text='GPS Ponto 3 [x,y]: ',bg='#fae5d3',padx=5,pady=5,font='Helvetica 10 bold')
    texto_GPS_Final.grid(column=3,row=2,padx=10,pady=10)
    GPS_Final = Label(tk, text=gps_xy_final[0],bg='#fdf2e9')
    GPS_Final.grid(column=3,row=3)
    GPS_Final = Label(tk, text=gps_xy_final[1],bg='#fdf2e9')
    GPS_Final.grid(column=3,row=4)
    
    texto_GPS_Satelites = Label(tk, text='Satélites disponíveis: ',bg='#f8f9f9',padx=5,pady=5,font='Helvetica 10 bold')
    texto_GPS_Satelites.grid(column=1,row=5,padx=10,pady=10)
    GPS_Satelites = Label(tk, text=n_satelites,bg='#FFF')
    GPS_Satelites.grid(column=1,row=6)
    
    texto_correcaoErro = Label(tk, text='Correção em Angulo (Erro): ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
    texto_correcaoErro.grid(column=1,row=7,padx=10,pady=10)
    correcaoErro = Label(tk, text=erro,bg='#FFF')
    correcaoErro.grid(column=1,row=8)
    
    texto_correcaoGiro = Label(tk, text='Sinal de Giro (esq/dir = +/-): ',bg='#fff',padx=5,pady=5,font='Helvetica 10 bold')
    texto_correcaoGiro.grid(column=2,row=7,padx=10,pady=10)
    
    if sinalGiro > 0:
        correcaoGiro = Label(tk, text='esquerda (+)',bg='#FFF')
        correcaoGiro.grid(column=2,row=8)
    else:
        correcaoGiro = Label(tk, text='direita (-)',bg='#FFF')
        correcaoGiro.grid(column=2,row=8)
        
    texto_GPS_Final_input = Label(tk, text='Final (Latitude): ',bg='#e59866',fg="#fff",padx=5,pady=5,font='Helvetica 10 bold')
    texto_GPS_Final_input.grid(column=1,row=9,padx=10,pady=10)
    GPS_Final_input_lat = Entry(tk)
    GPS_Final_input_lat.grid(column=1,row=10)
    
    texto_GPS_Final_input = Label(tk, text='Final (Longitude): ',bg='#e59866',fg="#fff",padx=5,pady=5,font='Helvetica 10 bold')
    texto_GPS_Final_input.grid(column=2,row=9,padx=10,pady=10)
    GPS_Final_input_lon = Entry(tk)
    GPS_Final_input_lon.grid(column=2,row=10)
    
    botaoPonto = Button(tk, text="Novo Ponto Final", command=SalvarNovoPontoFinal,bg='#dc7633',fg="#fff")
    botaoPonto.grid(column=3,row=10,padx=10,pady=10)
    
    botaoSair = Button(tk, text="Gráfico", command=GerarCSV,bg='#7fb3d5',fg="#fff")
    botaoSair.grid(column=2,row=11,padx=10,pady=10)
    
    botaoGrafico = Button(tk, text="Parar Motor", command=PararMotor,bg='#f1948a',fg="#fff")
    botaoGrafico.grid(column=2,row=12,padx=10,pady=10)
    
    botaoSair = Button(tk, text="PAUSAR", command=Pausar,bg='#c0392b',fg="#fff")
    botaoSair.grid(column=1,row=11,padx=10,pady=10)
    
    botaoSair = Button(tk, text="SAIR", command=Sair,bg='#c0392b',fg="#fff")
    botaoSair.grid(column=1,row=12,padx=10,pady=10)
    
    '''
    canvas = FigureCanvasTkAgg(fig, master=tk)
    canvas.get_tk_widget().grid(column=2,row=10,columnspan=4, rowspan=11,padx=10,pady=10)
    
    ani = animation.FuncAnimation(fig, animate, interval=20)
    '''
    tk.mainloop()
    
      
except(KeyboardInterrupt, SystemExit):
    #gpio.cleanup()
    
    thread.__stop()
    thread.join()
    time.sleep(5)
    pass
