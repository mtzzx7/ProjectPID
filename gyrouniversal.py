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
# Contador para rampa inicial de correção (evita pulso/zigzag nos primeiros ciclos)
moviment_call_count = 0


# Parâmetros do GyroTurn
GYRO_TOL = 5
GYRO_MIN = 40
GYRO_MAX = 60


def PID(kp, ki, kd, erro, integral, last_error, wait_func):
    """Implementação simples de PID usado nas rotinas de giroscópio."""
    integral += erro
    derivado = erro - last_error
    correction = kp * erro + ki * integral + kd * derivado
    last_error = erro
    wait_func(0)
    return correction, integral, last_error


def gyrouniversal(parametro, rotate_mode=None, kp=1.8, ki=0, kd=0.5):
    """Gira o robô em torno do seu eixo até atingir um ângulo alvo."""
    integral = 0
    last_error = 0
    correction = 0

    # kp, ki, kd are received as parameters (defaults provided in signature)

    hub.imu.reset_heading(0)
    alvo = parametro

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180
    erro = erro_angular(alvo, hub.imu.heading())

    # Detecta automaticamente qual sentido de potência produz giro positivo
    # (evita problemas quando a fiação/mapeamento dos motores tem convenção diferente).
    try:
        heading_before = hub.imu.heading()
        test_power = max(20, GYRO_MIN // 2)
        # pequeno pulso: right forward, left backward
        right_motor.dc(test_power)
        left_motor.dc(-test_power)
        wait(80)
        right_motor.brake()
        left_motor.brake()
        wait(40)
        heading_after = hub.imu.heading()
        delta_head = ((heading_after - heading_before + 540) % 360) - 180
        rotate_positive_sign = 1 if delta_head >= 0 else -1
    except Exception:
        rotate_positive_sign = 1

    while abs(erro) > GYRO_TOL:
        correction, integral, last_error = PID(kp, ki, kd, erro, integral, last_error, wait)
        # Determina magnitude da potência (sempre positiva) e aplica direção
        # com base no sinal da correção. Isso evita inversões de sentido
        # por causa de mudanças na convenção de sinais.
        pot_mag = abs(correction)
        # Limita magnitude entre mínimo e máximo definidos
        if pot_mag < GYRO_MIN:
            pot_mag = GYRO_MIN
        if pot_mag > GYRO_MAX:
            pot_mag = GYRO_MAX
        pot_mag = int(pot_mag)

        # Aplica potência preservando o sentido da correção (positive -> giro no sentido positivo)
        sign = 1 if correction >= 0 else -1
        right_motor.dc(sign * rotate_positive_sign * pot_mag)
        left_motor.dc(-sign * rotate_positive_sign * pot_mag)

        if rotate_mode == "calibrar":
            ler_e_atualizar()
        # A lógica de 'else' para 'rotate_mode' diferente de 'calibrar' ou 'None' foi removida
        # pois o comportamento de giro no eixo é o mesmo. Se precisar de um giro
        # com uma roda parada, seria melhor criar uma função separada.

        erro = erro_angular(alvo, hub.imu.heading())

    right_motor.hold()
    left_motor.hold()

    erro_residual = erro_angular(alvo, hub.imu.heading())
    if abs(erro_residual) > 5:
        pulso_pot = 40
        if erro_residual > 0:
            right_motor.dc(pulso_pot)
            left_motor.dc(-pulso_pot)
        else:
            right_motor.dc(-pulso_pot)
            left_motor.dc(pulso_pot)
        right_motor.hold()
        left_motor.hold()





def moviment(kp, ki, kd, erro, integral, last_error, wait, speed):
    """Função auxiliar que aplica controle PID simples para manter heading = 0.

    Retorna o erro, integral e last_error atualizados para preservação do estado
    do PID entre iterações.
    """
    angulo_atual = hub.imu.heading()
    erro = angulo_atual - 0
    correction, integral, last_error = PID(kp, ki, kd, erro, integral, last_error, wait)
    left_power = speed + correction
    right_power = speed - correction

    left_motor.dc(left_power)
    right_motor.dc(right_power)

    return erro, integral, last_error


def gyro_move_universal(mode, velocidade, parametro=None, kp=-2, ki=0.001, kd=0.6):
    """Método universal para movimentação com diferentes critérios de parada.

    Os ganhos `kp`, `ki`, `kd` têm valores padrão mais conservadores adequados
    para iniciar testes (reduzem respostas agressivas que causam zigzag/giro
    ao mover para ré). Ajuste conforme necessário quando testar no robô.
    """
    redefinir()
    erro = 0
    integral = 0
    last_error = 0
    # kp, ki, kd can be tuned via parameters (defaults chosen conservatively)
    hub.imu.reset_heading(0)
    robot.reset(0, 0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    if mode == "dois_pretos":
        while color_sensor_esquerdo.reflection() > 20 and color_sensor_direito.reflection() > 20:
            erro, integral, last_error = moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
        print("Hello Word")
        left_motor.brake()
        right_motor.brake()
    elif mode == "um_preto":
        while color_sensor_esquerdo.reflection() > 20 or color_sensor_direito.reflection() > 20:
            erro, integral, last_error = moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
        left_motor.brake()
        right_motor.brake()
    
    elif mode == "angulo":
        while True:
            erro, integral, last_error = moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
            # calcula a rotação média dos dois motores corretamente
            rot_atual = (left_motor.angle() + right_motor.angle()) / 2
            distancia_atual = abs(rot_atual) / 360 * (2 * 3.14159 * (55 / 10))
            if abs(distancia_atual) >= abs(parametro):
                left_motor.hold()
                right_motor.hold()
                break
            print(hub.imu.heading())

    """
    elif mode == "distancia":
        while True:
            erro, integral, last_error = moviment(kp, ki, kd, erro, integral, last_error, wait, velocidade)
            if ultra.distance() <= parametro:
                break
        left_motor.brake()
        right_motor.brake()
    """

def calibrar():
    global GYRO_MAX, GYRO_MIN
    GYRO_MAX = 100
    GYRO_MIN = 40
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


def lqr_move(distancia_alvo_cm, potencia_base=50):
    """
    Move o robô em linha reta usando um controlador LQR para manter a direção (guinada).

    Esta função utiliza o giroscópio (IMU) para ler o ângulo de guinada (heading)
    e sua taxa de variação. Um controlador LQR calcula a correção necessária
    para manter o robô andando em linha reta (heading = 0).

    AVISO IMPORTANTE:
    A matriz de ganhos 'K' abaixo é um EXEMPLO. Para que o robô funcione
    corretamente, você PRECISA AJUSTAR esses ganhos experimentalmente para
    a massa, geometria e motores do SEU robô.

    Args:
        distancia_alvo_cm (float): Distância que o robô deve percorrer em cm.
        potencia_base (int): Potência base (de 0 a 100) a ser aplicada aos motores.
    """
    # --- INÍCIO DA CONFIGURAÇÃO LQR PARA GUINADA (HEADING) ---

    # ATENÇÃO: ESTA MATRIZ DE GANHO 'K' É APENAS UM EXEMPLO!
    # VOCÊ PRECISA AJUSTAR OS VALORES PARA O SEU ROBÔ!
    # Formato: [ganho_angulo_guinada, ganho_taxa_guinada]
    # - ganho_angulo_guinada (K[0]): Reage ao erro de ângulo. Um valor maior
    #   fará o robô corrigir a direção de forma mais agressiva.
    # - ganho_taxa_guinada (K[1]): Amortece a correção, evitando oscilações.
    #   Um valor maior torna o robô mais "suave" ao voltar para a linha reta.
    K = [1.14, 1]  # Ponto de partida para ajuste

    # Constantes do robô (ajuste conforme necessário)
    RAIO_RODA_MM = 55 # Raio da roda em milímetros

    # --- FIM DA CONFIGURAÇÃO LQR ---

    # Inicialização de hardware e estado
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    # Loop de controle principal
    while True:
        # --- Verificação da Condição de Parada ---
        # Calcula a distância percorrida com base na rotação média dos motores
        motor_angle_deg = (left_motor.angle() + right_motor.angle()) / 2
        distancia_percorrida_cm = (motor_angle_deg / 360) * (2 * 3.14159 * (RAIO_RODA_MM / 10))
        if abs(distancia_percorrida_cm) >= abs(distancia_alvo_cm):
            break

        # --- Lógica do Controlador LQR para Guinada ---
        # O objetivo é manter a guinada (heading) em 0 para andar em linha reta.
        # O LQR usa o "State-Feedback Control": u = -Kx
        # onde 'u' é a correção e 'x' é o vetor de estado [ângulo, taxa_do_ângulo].

        # 1. Leitura dos Sensores para montar o vetor de estado 'x'
        # Estado 1: Ângulo de guinada (heading)
        heading_angle = hub.imu.heading()
        # Estado 2: Taxa de variação da guinada (velocidade angular no eixo Z)
        # O índice [2] corresponde ao eixo Z (vertical), que é a guinada.
        heading_rate = hub.imu.angular_velocity()[2]

        # 2. Calcular a saída de controle 'u' (a correção)
        # Multiplica cada estado pelo seu ganho correspondente e soma tudo.
        correction = -(K[0] * heading_angle + K[1] * heading_rate)

        # 3. Aplicar a potência base e a correção aos motores
        # A correção é subtraída de um lado e somada ao outro para girar o robô.
        left_power = potencia_base - correction
        right_power = potencia_base + correction

        # Limita a potência para a faixa segura do motor [-100, 100]
        left_power = max(-100, min(100, left_power))
        right_power = max(-100, min(100, right_power))

        left_motor.dc(left_power)
        right_motor.dc(right_power)

        wait(10)  # Pequena pausa para não sobrecarregar o loop

    # Parar os motores ao final
    left_motor.brake()
    right_motor.brake()