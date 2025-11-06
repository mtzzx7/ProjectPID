# Resgate - Configurações de hardware e parâmetros globais
#
# Este arquivo centraliza a inicialização do hardware usado pelo projeto "Resgate".
# Contém as chamadas de criação dos objetos Pybricks (hub, motores, sensores)
# e parâmetros físicos do robô (diâmetro das rodas e distância entre elas).

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


# --- INICIALIZAÇÃO DO HARDWARE ---
# Cria o objeto principal do hub. Usado para acessar IMU (giroscópio/compasso),
# botões, display, speaker, etc.
hub = PrimeHub()

# Motores das rodas: ajuste portas/ direção conforme montagem física.
# - left_motor: roda esquerda conectada na porta A
# - right_motor: roda direita conectada na porta B
# Direction.CLOCKWISE / COUNTERCLOCKWISE ajusta sentido positivo da rotação
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)

# Motor da garra (manipulador) - porta E neste projeto

# Sensor ultrassônico usado para detecção de objetos/distância - porta F
ultra = UltrasonicSensor(Port.E)

# Sensores de cor (reflexão) utilizados pelo seguidor de linha
# - sensor direito: porta D
# - sensor esquerdo: porta C
color_sensor_direito = ColorSensor(Port.C)
color_sensor_esquerdo = ColorSensor(Port.D)


# Parâmetros físicos do robô (importante para o DriveBase)
# - wheel_diameter: diâmetro da roda em mm. Ajustar para sua roda.
# - axle_track: distância entre os eixos das rodas (center-to-center) em mm.
wheel_diameter = 55  # Ajuste este valor para o diâmetro real da sua roda (mm)
axle_track = 164     # Ajuste este valor para a distância real entre as rodas (mm)

# DriveBase provê métodos de alto nível para controlar o deslocamento do robô
# (straight, turn, drive) usando as instâncias de motor e parâmetros físicos.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

# Nota: variáveis como 'hub', 'left_motor', 'right_motor', 'robot', etc. são
# importadas por outros módulos do projeto e, portanto, devem permanecer
# no escopo global deste módulo (não encapsular em função/classe aqui).