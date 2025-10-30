from configuracao import *
from gyrouniversal import *


class LineFollower:
    """Classe que implementa um seguidor de linha com controle PID.

    A arquitetura separa:
    - calibração (medir branco e preto)
    - loop de controle PID que mantém o robô centralizado na linha

    Parâmetros ajustáveis:
    - kp, ki, kd: ganhos do controlador
    - limites de potência aplicados às rodas (clamp em calculate_pid)
    """

    def __init__(self):
        # Ganhos PID (podem ser ajustados conforme comportamento em pista)
        self.kp = 1.3   # proporcional
        self.ki = 0  # integral
        self.kd = 0.0035  # derivativo
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.error = 0
        self.error_history = []
        self.max_history_length = 5  # comprimento do histórico para detectar oscilações
        self.is_calibrated = False
        self.timer = StopWatch()
        self.ordem = 0 # estado para sequência de ações (manipulador, etc.)
        self.qnt_pretos = 0  # contador de leituras consecutivas em preto


    def calibracao(self):
        """Processo de calibração manual.

        Fluxo esperado:
        1) Colocar robô sobre branco e pressionar botão central
        2) Ler refletância dos sensores como referência de 'branco'
        3) Girar/mover para linha preta e ler refletância como referência de 'preto'

        O método armazena os valores max/min com margens para evitar ruído.
        """
        print("Iniciando Calibração")
        print("Coloque o robô sobre a superfície branca e pressione o botão central")
        hub.display.char("B")
        # Leituras iniciais para branco
        reflexo_branco_dir = color_sensor_direito.reflection()
        reflexo_branco_esq = color_sensor_esquerdo.reflection()

        # Averigua algumas leituras para estabilizar
        for _ in range(2):
            reflexo_branco_dir = color_sensor_direito.reflection()
            reflexo_branco_esq = color_sensor_esquerdo.reflection()
        print("\nBranco calibrado.")
        hub.display.char("P")
        hub.speaker.beep(24, 100)
        print("Coloque o robô sobre a linha preta e pressione o botão central")

        # Leituras para preto — o código original executa uma sequência de
        # movimentos para localizar a linha antes de capturar o valor.
        reflexo_preto_dir = color_sensor_direito.reflection()
        reflexo_preto_esq = color_sensor_esquerdo.reflection()
        gyrouniversal(-90)
        print("Virou para esquerda")
        gyro_move_universal('dois_pretos', -40)
        print("Chegou na linha preta")
        left_motor.stop()
        right_motor.stop()
        for _ in range(2):
            reflexo_preto_dir = color_sensor_direito.reflection()
            reflexo_preto_esq = color_sensor_esquerdo.reflection()
            print(f"Refletância atual - Esq: {reflexo_preto_esq}% Dir: {reflexo_preto_dir}%", end='\r')
        gyro_move_universal(0, 40, 0.5)
        gyrouniversal(90)
        print("\nPreto calibrado.")

        hub.display.off()

        # Aplica margens para evitar regiões muito próximas (ruído)
        self.reflexo_preto_dir = reflexo_preto_dir + 5
        self.reflexo_branco_dir = reflexo_branco_dir - 5
        self.reflexo_preto_esq = reflexo_preto_esq + 5
        self.reflexo_branco_esq = reflexo_branco_esq - 5
        self.is_calibrated = True

        print("Calibração concluída")
        print(f"Sensor Direito - Min: {self.reflexo_branco_esq}%, Max: {self.reflexo_preto_dir}%")
        print(f"Sensor Esquerdo - Min: {self.reflexo_branco_esq}%, Max: {self.reflexo_branco_dir}%")

    def calculate_pid(self):
        """Loop principal do controlador PID do seguidor de linha.

        Estrutura:
        - normaliza leituras dos sensores (0..100)
        - calcula erro (esquerda - direita)
        - aplica termos P, I, D
        - limita potência e aplica aos motores

        Critérios de parada/ações especiais:
        - usa `ultra` para detectar obstáculos e disparar sequência de movimentos
        - conta leituras consecutivas em preto (qnt_pretos) para detectar cruzamentos
        - interrompe quando qnt_pretos >= 12 ou botão central pressionado
        """
        # Executa até detectar uma sequência longa de preto ou até botão central
        while Button.CENTER not in hub.buttons.pressed():
            try:
                # Normaliza entre 0 e 100 baseado nas leituras calibradas
                self.norm_dir = normaliza(color_sensor_direito.reflection(), self.reflexo_preto_dir, self.reflexo_branco_dir)
                self.norm_esq = normaliza(color_sensor_esquerdo.reflection(), self.reflexo_preto_esq, self.reflexo_branco_esq)
            except Exception:
                # Se não calibrado ou erro, usa leitura direta (fallback)
                self.norm_esq = color_sensor_esquerdo.reflection()
                self.norm_dir = color_sensor_direito.reflection()

            # Erro positivo => mais reflexo à esquerda (ajustar para direita)
            self.error = self.norm_esq - self.norm_dir
            self.integral += self.error
            self.derivative = self.error - self.last_error
            self.last_error = self.error
            self.error_history.append(self.error)
            if len(self.error_history) > self.max_history_length:
                self.error_history.pop(0)

            # Anti-windup simples para integral
            if self.integral > 100:
                self.integral = 0
            elif self.integral < -100:
                self.integral = 0

            # Cálculo do PID e clamp das potências de saída
            correction = (self.kp * self.error) + (self.kd * self.derivative)
            left_power = max(min(40 + correction, 100), -100)
            right_power = max(min(40 - correction, 100), -100)
            left_motor.dc(left_power)
            right_motor.dc(right_power)
            # Sequência de ação ao detectar obstáculo próximo pelo ultrassom
            if ultra.distance() <= 60 and self.ordem == 0:
                # Para e realiza manobra de busca/recuperação
                left_motor.stop()
                right_motor.stop()
                gyro_move_universal(0, -50, 0.4)
                gyrouniversal(90)
                gyro_move_universal(0, 50, 2.7)
                gyrouniversal(-90)
                gyro_move_universal(0, 50, 1.4)
                gyrouniversal(90)
                gyro_move_universal(0, 50, 0.5)
                self.ordem = 1

            elif ultra.distance() <= 45 and self.ordem == 1:
                # Aciona a garra para pegar objeto com movimento preciso
                left_motor.stop()
                right_motor.stop()
                gyro_move_universal(0, 50, 0.2)
                # Usa o novo método para movimento preciso da garra
                wait(500)

                self.kp = 1.4
                self.ki = 0.00001
                self.kd = 0.1
                # Reset do estado do controlador para prevenir correções abruptas
                self.integral = 0
                self.derivative = 0
                self.last_error = 0
            wait(10)  # pequeno delay para evitar loop apertado
        left_motor.stop()
        right_motor.stop()


# Função global para normalizar leitura dos sensores
def normaliza(reflection, preto, branco):
    """Normaliza a leitura de refletância para uma escala 0..100.

    - reflection: leitura atual do sensor
    - preto: valor medido durante calibração para preto (mínimo)
    - branco: valor medido durante calibração para branco (máximo)

    Retorna 0 se branco == preto (evita divisão por zero).
    """
    if branco == preto:
        return 0
    return (reflection - preto) / (branco - preto) * 100


def main():
    gyro_move_universal("angulo", 50, 100)

if __name__ == '__main__':
    main()