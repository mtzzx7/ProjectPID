#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor,OUTPUT_B,OUTPUT_C,SpeedPercent,MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_3
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sound import Sound
from ev3dev2.button import Button
import time, csv

log_on = True

def sinal(a):
    return 1 if a>=0 else -1

if log_on:
    arquivo_csv = open('//home//robot//PID_SeguidorLinha.csv','w',newline='')
    escritor_csv = csv.writer(arquivo_csv)
    escritor_csv.writerow(['Tempo', 'P', 'I', 'D'])
    log_data = {"tempo": [], "P": [], "I": [], "D": [],"W":[]}

    def log(mensagem):
        escritor_csv.writerow(mensagem[:])




sound = Sound()
sound.beep()

motor_esq = LargeMotor(OUTPUT_B)
motor_dir = LargeMotor(OUTPUT_C)

cor = ColorSensor(INPUT_3)
botao = Button()

MAX = 70
min = 10
objetivo = (MAX+min)/2

v=50

kp=10
ki=2
kd = 0

dt = 1E-6
I= 0

t = ti = time.perf_counter()

erro_i = erro = objetivo - (100 - cor.reflected_light_intensity)

while not(botao.any()):
    continue

while not(botao.any()):
    
    P = kp*erro
    I += ki*erro*dt
    D = kd*(erro-erro_i)/dt
    
    w = P+I+D
    w = sinal(erro)*max(100,w)

    vr = (w+2*v)/2
    ve = 2*v -vr
    
    if log_on:
        log_data["tempo"].append(t-ti)
        log_data["P"].append(P)
        log_data["I"].append(I)
        log_data["D"].append(D)
        log_data["w"].append(w)

    t=time.time()
            
    motor_dir.on(vr)
    motor_esq.on(ve)
            

    
  
for i in range(len(log_data["tempo"])):
 
    log([log_data['tempo'][i],log_data['P'][i],log_data['I'][i],log_data['D'][i]])

motor_esq.stop()
motor_dir.stop()
sound.beep()


arquivo_csv.close()
input("Pressione Enter para encerrar...")