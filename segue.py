from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Button
from pybricks.tools import wait, StopWatch
from configuracao import *

class LineFollower:
    """Seguidor de linha usando dois sensores de cor com controle PID."""
    def __init__(self, alpha=0.8):
        # Ganhos PID (ajustáveis)
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        # Estados do PID
        self.integral = 0
        self.last_error = 0
        # Calibração de reflexo
        self.black_esq = 0
        self.white_esq = 0
        self.black_dir = 0
        self.white_dir = 0
        # Filtro exponencial
        self.alpha = alpha
        self.smoothed_left = 0
        self.smoothed_right = 0

    def calibrate(self):
        """Calibra branco e preto para ambos sensores (pressionar central para confirmar)."""
        print("Posicione os sensores sobre a linha PRETA e pressione o botão central.")
        # Aguarda o usuário colocar sobre preto
        while Button.LEFT not in hub.buttons.pressed():
            wait(10)
        self.black_esq = color_sensor_esquerdo.reflection()
        self.black_dir = color_sensor_direito.reflection()
        wait(300)
        print(f"Valor preto L: {self.black_esq}, R: {self.black_dir}")

        print("Agora posicione sobre o fundo BRANCO e pressione o botão central.")
        while Button.LEFT not in hub.buttons.pressed():
            wait(10)
        self.white_esq = color_sensor_esquerdo.reflection()
        self.white_dir = color_sensor_direito.reflection()
        wait(300)
        print(f"Valor branco L: {self.white_esq}, R: {self.white_dir}")

        # Inicializa filtro com os valores de início (branco)
        self.smoothed_left = 100.0
        self.smoothed_right = 100.0

    def normalize(self, value, black, white):
        """Normaliza leitura (preto->0, branco->100):contentReference[oaicite:12]{index=12}."""
        if white == black:
            return 0.0
        return (value - black) / (white - black) * 100

    def follow(self, base_speed: float):
        """
        Segue a linha até detectar cruzamento (ambos sensores no preto) ou parada manual.
        Aplica filtro exponencial e PID para correção de trajetória.
        """
        watch = StopWatch()
        watch.reset()

        while True:
            # Leitura bruta dos sensores
            raw_left = color_sensor_esquerdo.reflection()
            raw_right = color_sensor_direito.reflection()

            # Normalização de 0 a 100:contentReference[oaicite:13]{index=13}
            norm_left = self.normalize(raw_left, self.black_esq, self.white_esq)
            norm_right = self.normalize(raw_right, self.black_dir, self.white_dir)

            # Filtro exponencial para suavizar
            self.smoothed_left = self.alpha * self.smoothed_left + (1 - self.alpha) * norm_left
            self.smoothed_right = self.alpha * self.smoothed_right + (1 - self.alpha) * norm_right

            # Calcula erro (esquerda - direita) para manter robô centralizado
            error = self.smoothed_left - self.smoothed_right

            # Calcula PID
            dt = watch.time() / 1000  # tempo em segundos
            watch.reset()
            # Proteção contra dt muito pequeno
            if dt <= 0:
                dt = 0.01
            self.integral += error * dt
            derivative = (error - self.last_error) / dt
            self.last_error = error

            correction = self.kp * error + self.ki * self.integral + self.kd * derivative

            # Calcula potências dos motores
            power_left = base_speed - correction
            power_right = base_speed + correction
            # Limita potências
            power_left = max(-100, min(100, int(power_left)))
            power_right = max(-100, min(100, int(power_right)))

            # Aplica potência
            left_motor.dc(power_left)
            right_motor.dc(power_right)

            # Condição de parada: cruzamento (ambos perto de preto) ou botão
            # Se ambos sensores estiverem detectando preto (valor baixo normalizado)
            if norm_left <= 5 and norm_right <= 5:
                print("Cruzamento detectado!")
                break
            if Button.CENTER in hub.buttons.pressed():
                print("Parada manual pelo botão.")
                break

            wait(10)

        # Para motores no final
        left_motor.brake()
        right_motor.brake()

line = LineFollower()
line.calibrate()
line.follow(50)

