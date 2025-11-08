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

def mover(vr,ve):
    motor_dir.on(vr)
    motor_esq.on(ve)
    
def media(rgb):
    return sum(rgb)/3

def limita(vel):
    
    return sinal(vel)*min(abs(vel),100)

def escala(valor):
    return valor*MAX/255

    

if log_on:
    
    arquivo_csv = open('//home//robot//PID_SeguidorLinha.csv','w',newline='')
    escritor_csv = csv.writer(arquivo_csv)
    escritor_csv.writerow(['Tempo','Erro'])
    log_data = {"tempo": [],"Erro":[]}
    
    def log(mensagem):

        escritor_csv.writerow(mensagem[:])
    
    def dados(tempo):
        # Adiciona o tempo decorrido desde ti
        log_data["tempo"].append(tempo)
        log_data["Erro"].append(erro)

sound = Sound()
sound.beep()

motor_esq = LargeMotor(OUTPUT_B)
motor_dir = LargeMotor(OUTPUT_C)

cor = ColorSensor(INPUT_3)
botao = Button()

MAX = 50
MIN = 0
objetivo = (MAX+MIN)/2

kp = 3
ki = 0.5
kd = 0.03

erro = erro_anterior = escala(objetivo-media(cor.rgb))

P = kp*erro
I = 0
D = 0

dt = lambda : time.perf_counter()-t

w = P+I+D
v = 7
vd = limita(v + w)
ve = limita(2*v-vd)

ti = time.perf_counter()

while not(botao.any()):
        
    t=time.perf_counter()
    mover(vd,ve)
    dados(t)
        
    # while time.perf_counter()-t<dt(): continue
    
    erro_anterior = erro
    erro = escala(objetivo-media(cor.rgb))
    P = kp*erro
    I += ki*erro*dt()
    D = kd*(erro - erro_anterior)/dt()
    
    w = P+I+D
    vd = limita(v + w)
    ve = limita(2*v-vd)

motor_esq.stop()
motor_dir.stop()

if log_on:
    log_data["tempo"].append(time.perf_counter())
    log_data["Erro"].append(media(cor.rgb))
    
    for i in range(len(log_data["tempo"])):
        log([log_data['tempo'][i]-ti,log_data['Erro'][i]*255/MAX])

    arquivo_csv.close()
sound.beep()


input("Pressione Enter para encerrar...")