#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor,OUTPUT_B,OUTPUT_C,SpeedPercent,MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor,GyroSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.button import Button
import time, csv

log_on = True

def girar(velocidade):
    motor_dir.on(-velocidade)
    motor_esq.on(velocidade)

def sinal(a):
    return 1 if a>=0 else -1

if log_on:
    
    arquivo_csv = open('//home//robot//PID_giroscopio.csv','w',newline='')
    escritor_csv = csv.writer(arquivo_csv)
    escritor_csv.writerow(['Tempo', 'P', 'I', 'D','Velocidade','Angulo'])
    log_data = {"tempo": [], "P": [], "I": [], "D": [],"U":[],"Ang":[]}
    
    def log(mensagem):

        escritor_csv.writerow(mensagem[:])
    
    def dados(tempo):
        # Adiciona o tempo decorrido desde ti
        log_data["tempo"].append(tempo)
        log_data["P"].append(P)
        log_data["I"].append(I)
        log_data["D"].append(D)
        log_data["U"].append(U)
        log_data["Ang"].append(giro.angle)
        

sound = Sound()
sound.beep()

motor_esq = LargeMotor(OUTPUT_B)
motor_dir = LargeMotor(OUTPUT_C)

giro = GyroSensor(INPUT_2)
giro.mode = giro.MODE_GYRO_ANG
giro.reset()

angulo_alvo = -720
tol = 1

erro = erro_i = angulo_alvo-giro.angle


kp = 1
ki = 0.1
kd = 0.01
dt = 0.01

P = kp*erro
I=0
D = kd*(erro-erro_i)/dt
U = P+I+D
U = sinal(erro)*min(abs(U)*100/720,100)

ti = time.perf_counter()


while abs(erro)>tol:
    t = time.perf_counter()
    dados(t)
    girar(U)
    erro_i=erro
    while(time.perf_counter()-t<dt):
        continue
    
    erro = angulo_alvo-giro.angle
    P = kp*erro
    I += ki*erro*dt
    D = kd*(erro-erro_i)/dt
    U = P+I+D
    
    U = sinal(erro)*min(abs(U)*100/360,100)
        
motor_esq.stop()
motor_dir.stop()

t = time.perf_counter()
dados(t)

    
if log_on:
    log_data["tempo"].append(time.perf_counter())
    log_data["P"].append(P)
    log_data["I"].append(I)
    log_data["D"].append(D)
    log_data["U"].append(U)
    log_data["Ang"].append(giro.angle)
    for i in range(len(log_data["tempo"])):
    
        log([log_data['tempo'][i]-ti,log_data['P'][i],log_data['I'][i],log_data['D'][i],log_data['U'][i],log_data['Ang'][i]])
    arquivo_csv.close()
          
sound.beep()

input("Pressione Enter para encerrar...")