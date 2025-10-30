"""
gyro.py
Movimentos e controle baseado em giroscópio (IMU) para o projeto Resgate.

Este módulo fornece funções para:
- GyroTurn: girar o robô com precisão usando a IMU do hub
- gyro_move: mover em linha reta mantendo um ângulo alvo
- gyro_universal: rotina genérica de movimento com diferentes critérios de parada

Observação: este módulo importa objetos do `config.py` (hub, motores, sensores,
robot) para reutilizar a inicialização de hardware feita lá.
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from configuracao import *


def redefinir():
    """Zera variáveis de estado do controlador PID/Gyro."""
    global last_error, integral, derivado, correcao, erro, methodstop
    last_error = 0
    integral = 0
    derivado = 0
    correction = 0
    erro = 0
    methodstop = 0


# Estado global (inicialização)
erro = 0
correction = 0
integral = 0
derivado = 0
last_error = 0
min_esq = 100
max_esq = 0
min_dir = 100
max_dir = 0


# Parâmetros do GyroTurn
GYRO_TOL = 0.05
GYRO_MIN = 30
GYRO_MAX = 70


def PID(kp, ki, kd, erro, integral, last_error, wait_func):
    """Implementação simples de PID usado nas rotinas de giroscópio."""
    integral += erro
    derivado = erro - last_error
    correction = kp * erro + ki * integral + kd * derivado
    last_error = erro
    wait_func(0)
    return correction, integral, last_error


def gyrouniversal(parametro , rotate_mode=None):
    """Gira o robô em torno do seu eixo até atingir um ângulo alvo."""
    integral = 0
    last_error = 0
    correction = 0

    kp = 6
    ki = 0.0008
    kd = 0.16

    hub.imu.reset_heading(0)
    alvo = parametro

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180
    erro = erro_angular(alvo, hub.imu.heading())

    while abs(erro) > GYRO_TOL:
        correction, integral, last_error = PID(kp, ki, kd, erro, integral, last_error, wait)
        potencia = int(min(GYRO_MAX, max(GYRO_MIN, abs(correction))))
        if rotate_mode == "calibrar":
            ler_e_atualizar()
        if rotate_mode == None:
            if erro > 0:
                right_motor.dc(-potencia)
                left_motor.dc(potencia)
            else:
                right_motor.dc(potencia)
                left_motor.dc(-potencia)
            
        elif rotate_mode == "calibrar":
            if erro > 0:
                right_motor.dc(-potencia)
                left_motor.dc(potencia)
            else:
                right_motor.dc(potencia)
                left_motor.dc(-potencia)

        else:
            if erro > 0:
                right_motor.dc(0)
                left_motor.dc(potencia)
            else:
                right_motor.dc(potencia)
                left_motor.dc(0)
        erro = erro_angular(alvo, hub.imu.heading())

    right_motor.brake()
    left_motor.brake()

    erro_residual = erro_angular(alvo, hub.imu.heading())
    if abs(erro_residual) > 0.2:
        pulso_pot = 5
        if erro_residual > 0:
            right_motor.dc(pulso_pot)
            left_motor.dc(-pulso_pot)
        else:
            right_motor.dc(-pulso_pot)
            left_motor.dc(pulso_pot)
        right_motor.brake()
        left_motor.brake()



def moviment(kp, ki, kd, erro, integral, last_error, wait, speed):
    """Função auxiliar que aplica controle PID simples para manter heading = 0."""
    angulo_atual = hub.imu.heading()
    erro = angulo_atual - 0
    correction, integral, last_error = PID(kp, ki, kd, erro, integral, last_error, wait)
    left_motor.dc(speed - correction)
    right_motor.dc(speed + correction)
    last_error = erro
    wait(10)


def gyro_move_universal(mode, velocidade, parametro=None):
    """Método universal para movimentação com diferentes critérios de parada."""
    redefinir()
    erro = 0
    integral = 0
    last_error = 0
    kp = 2
    ki = 0.001
    kd = 0.8
    hub.imu.reset_heading(0)
    robot.reset(0, 0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    if mode == "dois_pretos":
        while color_sensor_esquerdo.reflection() > 20 and color_sensor_direito.reflection() > 20:
            moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
        print("Hello Word")
        left_motor.brake()
        right_motor.brake()
    elif mode == "um_preto":
        while color_sensor_esquerdo.reflection() > 20 or color_sensor_direito.reflection() > 20:
            moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
        left_motor.brake()
        right_motor.brake()
    elif mode == "distancia":
        while True:
            moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
            if ultra.distance() <= parametro:
                break
        left_motor.brake()
        right_motor.brake()
    elif mode == "angulo":
        while True:
            moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
            rot_atual = abs(left_motor.angle() / 360) + abs(right_motor.angle() / 360) / 2
            distancia_atual = rot_atual * 17.27
            if distancia_atual >= abs(parametro):
                break

def calibrar():
    global GYRO_MAX, GYRO_MIN
    GYRO_MAX = 100
    GYRO_MIN = 70
    gyrouniversal(45, "calibrar")
    gyrouniversal(-90, "calibrar")
    gyrouniversal(45, "calibrar")
    GYRO_MIN = 20
    GYRO_MAX = 80


            

def ler_e_atualizar():
    global min_esq, max_esq, min_dir, max_dir
    ref_esq = color_sensor_esquerdo.reflection()
    ref_dir = color_sensor_direito.reflection()
    min_esq = min(min_esq, ref_esq)
    max_esq = max(max_esq, ref_esq)
    min_dir = min(min_dir, ref_dir)
    max_dir = max(max_dir, ref_dir)