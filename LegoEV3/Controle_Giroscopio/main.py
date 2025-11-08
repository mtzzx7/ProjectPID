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
    

sound = Sound()
sound.beep()

if log_on:
    
    sound.beep()
    arquivo_csv = open('//home//robot//giroscopio.csv','w',newline='')
    escritor_csv = csv.writer(arquivo_csv)
    escritor_csv.writerow(['Tempo', 'Velocidade','Angulo'])
    log_data = {"tempo": [], "vel": [], "ang": []}
    
    def log(mensagem):

        escritor_csv.writerow(mensagem[:])
        
    def dados():
    # Adiciona o tempo decorrido desde ti
        log_data["tempo"].append(time.perf_counter())
        log_data["vel"].append(sinal(erro)*min(100,abs(erro)*100/360))
        log_data["ang"].append(giro.angle)
    
motor_esq = LargeMotor(OUTPUT_B)
motor_dir = LargeMotor(OUTPUT_C)

giro = GyroSensor(INPUT_2)
giro.mode = giro.MODE_GYRO_ANG
giro.reset()

angulo_alvo = 45
tol = 1
dt = 1E-6

erro = angulo_alvo-giro.angle


if log_on:
    
    sound.beep()
    ti=time.perf_counter()
    log_data["tempo"].append(ti)
    log_data["vel"].append(sinal(erro)*min(100,abs(erro)*100/360))
    log_data["ang"].append(giro.angle)
        
while abs(erro)>tol:
    t = time.perf_counter()
    girar(sinal(erro)*min(100,abs(erro)*100/360))
    
    dados()
    while time.perf_counter()-t<dt: continue
    
    erro = angulo_alvo-giro.angle
    
motor_esq.stop()
motor_dir.stop()
    
if log_on:  
    sound.beep()  
    for i in range(len(log_data["tempo"])):
    
        log([log_data['tempo'][i]-ti,log_data['vel'][i],log_data['ang'][i]])
        
    arquivo_csv.close()
              

sound.beep()


input("Pressione Enter para encerrar...")