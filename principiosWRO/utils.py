from config import *
class LineFollower:
    """
    Seguidor de linha com controle PID otimizado para LEGO Spike Prime.
    Recursos:
    - Calibração automática (preto/branco)
    - Controle PID com ajuste dinâmico
    - Filtro de erro para reduzir oscilações
    - Modo agressivo para curvas
    """

    def __init__(self, kp=1.0, ki=0.005, kd=0.08):
        # Ganhos PID
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Variáveis do PID
        self.integral = 0
        self.last_error = 0
        self.error_history = []

        # Timer para cálculo de dt
        self.timer = StopWatch()

        # Valores calibrados
        self.preto_esq = 20
        self.branco_esq = 80
        self.preto_dir = 20
        self.branco_dir = 80

    def calibrar(self, tempo=200):
        """
        Calibração automática dos sensores.
        Gira o robô para medir preto e branco.
        """
        print("Iniciando calibração...")
        hub.imu.reset_heading(0)
        min_esq, max_esq = 100, 0
        min_dir, max_dir = 100, 0

        # Gira para coletar dados
        self.timer.reset()
        while self.timer.time() < tempo:
            left_motor.dc(30)
            right_motor.dc(30)
            ref_esq = color_sensor_esquerdo.reflection()
            ref_dir = color_sensor_direito.reflection()
            min_esq = min(min_esq, ref_esq)
            max_esq = max(max_esq, ref_esq)
            min_dir = min(min_dir, ref_dir)
            max_dir = max(max_dir, ref_dir)
            wait(10)

        left_motor.stop()
        right_motor.stop()

        # Define valores calibrados
        self.preto_esq, self.branco_esq = min_esq, max_esq
        self.preto_dir, self.branco_dir = min_dir, max_dir
        print(f"Calibração concluída: Esq({self.preto_esq}-{self.branco_esq}), Dir({self.preto_dir}-{self.branco_dir})")

    def normalizar(self, valor, preto, branco):
        """Normaliza leitura para escala 0-100."""
        if branco == preto:
            return 0
        return (valor - preto) / (branco - preto) * 100

    def seguir_linha(self, velocidade_base=50, modo="agressivo"):
        """
        Loop principal do seguidor de linha.
        - velocidade_base: velocidade nominal (0-100)
        - modo: 'agressivo' ou 'suave'
        """
        self.timer.reset()
        while Button.CENTER not in hub.buttons.pressed():
            # Normaliza leituras
            norm_esq = self.normalizar(color_sensor_esquerdo.reflection(), self.preto_esq, self.branco_esq)
            norm_dir = self.normalizar(color_sensor_direito.reflection(), self.preto_dir, self.branco_dir)

            # Calcula erro
            erro = norm_esq - norm_dir

            # Filtro simples (média móvel)
            self.error_history.append(erro)
            if len(self.error_history) > 3:
                self.error_history.pop(0)
            erro_filtrado = sum(self.error_history) / len(self.error_history)

            # Calcula dt
            dt = self.timer.time() / 1000
            self.timer.reset()
            if dt < 0.02:  # clamp mínimo
                dt = 0.02

            # Ajuste dinâmico para curvas
            if modo == "agressivo" and abs(erro_filtrado) > 30:
                kd = self.ki * 0.001
            else:
                kd = self.kd

            # PID
            self.integral += erro_filtrado * dt
            derivativo = (erro_filtrado - self.last_error) / dt
            correcao = (self.kp * erro_filtrado) + (self.ki * self.integral) + (kd * derivativo)
            self.last_error = erro_filtrado

            # Aplica correção
            left_power = max(min(velocidade_base - correcao, 100), -100)
            right_power = max(min(velocidade_base + correcao, 100), -100)
            left_motor.dc(left_power)
            right_motor.dc(right_power)

            wait(10)

        left_motor.stop()
        right_motor.stop()
        print("Seguidor encerrado.")

# Exemplo de uso:
seguidor = LineFollower()
seguidor.calibrar()
seguidor.seguir_linha(velocidade_base=50, modo="agressivo")