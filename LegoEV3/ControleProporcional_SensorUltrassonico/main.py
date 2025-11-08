#!/usr/bin/env python3
import csv, time
from ev3dev2.motor import LargeMotor,OUTPUT_B,OUTPUT_C,SpeedPercent,MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sound import Sound

log_on = True

def mover(velocidade):
    motor_dir.on(velocidade)
    motor_esq.on(velocidade)

def sinal(a):
    return 1 if a>=0 else -1

sound = Sound()


if log_on:
    
    sound.beep()
    arquivo_csv = open('//home//robot//ultrassonico.csv','w',newline='')
    escritor_csv = csv.writer(arquivo_csv)
    escritor_csv.writerow(['Tempo', 'Velocidade','Distancia'])
    log_data = {"tempo": [], "vel": [], "dist": []}
    
    def log(mensagem):

        escritor_csv.writerow(mensagem[:])
        
    def dados():
    # Adiciona o tempo decorrido desde ti
        log_data["tempo"].append(time.perf_counter())
        log_data["vel"].append(sinal(erro)*min(100,abs(erro)*100/360))
        log_data["dist"].append(sonar.distance_centimeters)

motor_esq = LargeMotor(OUTPUT_B)
motor_dir = LargeMotor(OUTPUT_C)

sonar = UltrasonicSensor(INPUT_4)
sonar.mode = 'US-DIST-CM'

distancia_alvo = 10.0
erro = sonar.distance_centimeters - distancia_alvo
tol = 1
dt = 1E-6

if log_on:
    
    sound.beep()
    ti=time.perf_counter()
    log_data["tempo"].append(ti)
    log_data["vel"].append(erro)
    log_data["dist"].append(sonar.distance_centimeters)

while abs(erro)>tol:
    t = time.perf_counter()
    mover(sinal(erro)*min(100,abs(erro)*100/30))
    
    dados()
    while time.perf_counter()-t<dt: continue
    
    erro = sonar.distance_centimeters - distancia_alvo
    
motor_dir.stop()
motor_esq.stop()

if log_on:  
    sound.beep()  
    for i in range(len(log_data["tempo"])):
    
        log([log_data['tempo'][i]-ti,log_data['vel'][i],log_data['ang'][i]])
        
    arquivo_csv.close()

input("Pressione Enter para encerrar...")