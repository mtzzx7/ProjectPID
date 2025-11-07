from configuracao import *
from gyrouniversal import *
 # Importado para potenciais usos futuros, embora não estritamente necessário para esta implementação
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
        self.kp = 1.6 # proporcional
        self.ki = 0.007 # integral
        self.kd = 0.03 # derivativo
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.error = 0
        self.error_history = []
        self.max_history_length = 5 # comprimento do histórico para detectar oscilações
        self.is_calibrated = False
        self.timer = StopWatch()
        self.ordem = 0 # estado para sequência de ações (manipulador, etc.)
        self.qnt_pretos = 0  # contador de leituras consecutivas em preto
        # Limiares para detecção de curva (valores normalizados)
        self.black_threshold = 20
        self.white_threshold = 80
        self.timer = StopWatch()
        self.dt = 0
    def detect_oscillation(self):
        """
        Detecta se o robô está oscilando (mudando de direção rapidamente).
        """
        if len(self.error_history) < self.max_history_length:
            return False
        # Conta quantas vezes o sinal do erro mudou (cruzou o zero)
        sign_changes = 0
        for i in range(1, len(self.error_history)):
            if self.error_history[i] * self.error_history[i-1] < 0:
                sign_changes += 1
        # Se houver muitas mudanças de sinal, considera-se que está oscilando
        return sign_changes > self.max_history_length * 0.6
    def detect_sharp_curve(self):
        """
        Detecta uma curva acentuada com base em valores extremos dos sensores normalizados.
        Retorna "sharp_left", "sharp_right", ou None.
        """
        # Curva acentuada para a esquerda (sensor direito vê preto)
        if self.norm_dir < self.black_threshold and self.norm_esq > self.white_threshold:
            return "sharp_left"
        # Curva acentuada para a direita (sensor esquerdo vê preto)
        elif self.norm_esq < self.black_threshold and self.norm_dir > self.white_threshold:
            return "sharp_right"
        else:
            return None
    def calibracao(self):
        """Processo de calibração automático por "scan" com giroscópio.
        O robô gira para a esquerda e depois para a direita usando o giroscópio
        para garantir uma varredura controlada da superfície.
        """
        print("Iniciando calibração por scan com giroscópio...")
        hub.display.icon(Icon.RIGHT)
        min_esq, max_esq = 100, 0
        min_dir, max_dir = 100, 0
        calibrar()
        # --- Cálculo da Margem Dinâmica ---
        range_esq = max_esq - min_esq
        range_dir = max_dir - min_dir
        if range_esq < 20 or range_dir < 20:
            print("\n!!! FALHA NA CALIBRAÇÃO !!!")
            print("Diferença entre branco e preto muito pequena. Usando valores padrão.")
            self.reflexo_preto_esq, self.reflexo_branco_esq = 20, 80
            self.reflexo_preto_dir, self.reflexo_branco_dir = 20, 80
        else:
            margin_esq = range_esq * 0.1
            margin_dir = range_dir * 0.1
            self.reflexo_preto_esq = min_esq + margin_esq
            self.reflexo_branco_esq = max_esq - margin_esq
            self.reflexo_preto_dir = min_dir + margin_dir
            self.reflexo_branco_dir = max_dir - margin_dir
        self.is_calibrated = True
        print("\nCalibração por scan concluída!")
        print(f"Sensor Esquerdo (Preto/Branco): {self.reflexo_preto_esq:.1f}% / {self.reflexo_branco_esq:.1f}%")
        print(f"Sensor Direito (Preto/Branco): {self.reflexo_preto_dir:.1f}% / {self.reflexo_branco_dir:.1f}%")
    def calculate_pid(self, start_speed, speed_oscilation):
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

            self.dt = self.timer.time() / 1000 
            print(self.dt) # transforma ms -> segundos
            self.timer.reset()

            if self.dt <= 0:
                self.dt = 0.01  # proteção
            self.error = self.norm_esq - self.norm_dir
            self.proporcional = self.kp * self.error
            self.integral += self.error * self.dt
            self.derivative = (self.error - self.last_error)/self.dt
            self.last_error = self.error
            self.error_history.append(self.error)
            if len(self.error_history) > self.max_history_length:
                self.error_history.pop(0)

            # --- Detecção de Oscilação e Curva Acentuada ---
            if self.detect_oscillation():
                print("Oscilação detectada!")

            curve = self.detect_sharp_curve()
            # Ação para curva será tratada no cálculo de correção abaixo

            # Anti-windup para integral, evitando que o termo cresça indefinidamente
            integral_limit = 500  # Um valor empírico, pode ser ajustado
            if self.integral > integral_limit:
                self.integral = integral_limit
            elif self.integral < -integral_limit:
                self.integral = -integral_limit


            if curve == "sharp_left":
                print("Curva acentuada para a esquerda detectada!")
                left_motor.stop()
                right_motor.stop()
                self.integral = self.integral * 0.5
                self.speed = speed_oscilation   # Força uma correção alta para virar à esquerda
            elif curve == "sharp_right":
                print("Curva acentuada para a direita detectada!")
                left_motor.stop()
                right_motor.stop()
                self.integral = self.integral * 0.5
                self.speed = speed_oscilation # Força uma correção alta para virar à direita
            else:
                self.speed = start_speed 
            # Cálculo do PID e clamp das potências de saída
            correction = self.proporcional + (self.ki * self.integral) + (self.kd * self.derivative)

            # Lógica para curvas acentuadas: sobrepõe a correção do PID

            left_power = max(min(-self.speed - correction, 100), -100)
            right_power = max(min(-self.speed + correction, 100), -100)
            left_motor.dc(left_power)
            right_motor.dc(right_power)
            # Sequência de ação ao detectar obstáculo próximo pelo ultrassom
        #     if ultra.distance() <= 60 and self.ordem == 0:
        #         # Para e realiza manobra de busca/recuperação
        #         left_motor.stop()
        #         right_motor.stop()
        #         gyro_move_universal("angulo", -50, 20)
        #         gyrouniversal(90)
        #         gyro_move_universal("angulo", 50, 80)
        #         gyrouniversal(-90)
        #         gyro_move_universal("angulo", 50, 100)
        #         gyrouniversal(-90)
        #         gyro_move_universal("angulo", 50, 80)
        #         gyrouniversal(90)
        #         gyro_move_universal("angulo", 50, 20)
        #         self.ordem = 1
        #     wait(10)  # pequeno delay para evitar loop apertado
        # left_motor.stop()
        # right_motor.stop()
    def _calculate_profile_speed(self, current_abs_angle_rotated, total_abs_angle, startspeed, maxspeed, endspeed, accelerate_ratio, decelerate_ratio):
        """
        Calcula a velocidade alvo com base em um perfil de velocidade trapezoidal.
        current_abs_angle_rotated: ângulo absoluto acumulado rotacionado até o momento.
        total_abs_angle: ângulo absoluto total a ser rotacionado.
        startspeed, maxspeed, endspeed: magnitudes das velocidades (0-100).
        accelerate_ratio: fração do total_abs_angle para a fase de aceleração.
        decelerate_ratio: fração do total_abs_angle para a fase de desaceleração.
        """
        if total_abs_angle == 0:
            return 0 # Sem rotação, sem velocidade
        accelerate_angle_length = total_abs_angle * accelerate_ratio
        decelerate_angle_length = total_abs_angle * decelerate_ratio
        # Garante que as velocidades são não-negativas para o cálculo; o sinal é aplicado depois.
        startspeed = abs(startspeed)
        maxspeed = abs(maxspeed)
        endspeed = abs(endspeed)
        # Fase de aceleração
        if current_abs_angle_rotated < accelerate_angle_length:
            if accelerate_angle_length == 0: return startspeed # Evita divisão por zero se não houver fase de aceleração
            speed_range = maxspeed - startspeed
            progress = current_abs_angle_rotated / accelerate_angle_length
            return startspeed + speed_range * progress
        # Fase de desaceleração
        elif current_abs_angle_rotated > total_abs_angle - decelerate_angle_length:
            if decelerate_angle_length == 0: return maxspeed # Evita divisão por zero se não houver fase de desaceleração
            speed_range = maxspeed - endspeed
            progress = (total_abs_angle - current_abs_angle_rotated) / decelerate_angle_length
            return endspeed + speed_range * progress
        # Fase de velocidade constante
        else:
            return maxspeed
    def arcRotation(self, radius, angle, startspeed, maxspeed, endspeed, addspeed=0.3, brakeStart=0.7, stopMethod=None, generator=None, stop=True, parametro=None):
        """
        Função para fazer o robô descrever uma curva com um raio e ângulo especificados.
        Parameters
        -------------
        radius: o raio da curva que o robô deve seguir; medido a partir do caminho da roda interna.
                Tipo: Inteiro. Se 0, o robô fará uma curva de ponto (pivot turn).
        angle: o ângulo que o robô deve girar na curva. Positivo para virar à esquerda, negativo para virar à direita.
               Tipo: Inteiro.
        startspeed: A velocidade inicial do robô (magnitude, 0-100). Tipo: Inteiro.
        maxspeed: A velocidade máxima que o robô atinge (magnitude, 0-100). Tipo: Inteiro.
        endspeed: A velocidade final do robô (magnitude, 0-100). Tipo: Inteiro.
        addspeed: A fração do ângulo total durante a qual o robô acelera. Tipo: Float. Padrão: 0.3
        brakeStart: A fração do ângulo total na qual o robô começa a desacelerar. Tipo: Float. Padrão: 0.7
        stopMethod: Um objeto opcional com um método `loop()` que retorna True quando o robô deve parar. Padrão: None
        generator: Um gerador opcional que executa algo em paralelo durante a condução. Não implementado nesta versão. Padrão: None
        stop: Um booleano que determina se o robô deve parar os motores após a condução. Padrão: True
        """
        # Placeholder para cancelamento externo, se necessário (ex: um membro da classe self.cancel_flag)
        # if self.cancel_flag:
        #     print("Rotação em arco cancelada.")
        #     return
        # Calibração do giroscópio conforme o exemplo
        calibrated_angle = angle * (2400 / 2443)
        # Leitura inicial do giroscópio e ângulo alvo
        initial_heading = hub.imu.heading()
        target_heading = initial_heading + calibrated_angle
        # Rastreia o ângulo total rotacionado para o perfil de velocidade
        total_abs_angle_to_rotate = abs(calibrated_angle)
        current_abs_angle_rotated = 0.0
        last_heading = initial_heading
        # Determina a direção geral do movimento (para frente/para trás)
        movement_direction_sign = 1 if startspeed >= 0 else -1
        # Calcula os fatores de velocidade para as rodas interna e externa
        # 'radius' é o raio do caminho da roda interna.
        # 'axle_track' é a distância entre os centros das rodas, de configuracao.py.
        left_speed_factor = 1.0
        right_speed_factor = 1.0
        if radius == 0: # Curva de ponto (pivot turn)
            if angle > 0: # Curva de ponto para a esquerda
                left_speed_factor = -1.0 # Roda esquerda para trás
                right_speed_factor = 1.0 # Roda direita para frente
            else: # Curva de ponto para a direita
                left_speed_factor = 1.0 # Roda esquerda para frente
                right_speed_factor = -1.0 # Roda direita para trás
        else: # Rotação em arco normal (radius > 0)
            # R_inner = radius
            # R_outer = radius + axle_track
            speed_ratio_outer_inner = (radius + axle_track) / radius

            if angle > 0: # Virar à esquerda
                # Motor esquerdo é interno, Motor direito é externo
                left_speed_factor = 1.0
                right_speed_factor = speed_ratio_outer_inner
            else: # Virar à direita
                # Motor direito é interno, Motor esquerdo é externo
                right_speed_factor = 1.0
                left_speed_factor = speed_ratio_outer_inner
        loop = True
        while loop:
            # if self.cancel_flag: # Placeholder para cancelamento externo
            #     break

            # if generator is not None: # Placeholder para execução paralela do gerador
            #     try:
            #         next(generator)
            #     except StopIteration:
            #         generator = None # Gerador finalizado
            current_heading = hub.imu.heading()
            # Calcula a mudança de ângulo, tratando a passagem por 0/360 para o caminho mais curto
            heading_delta = (current_heading - last_heading + 540) % 360 - 180
            current_abs_angle_rotated += abs(heading_delta) # Acumula o ângulo absoluto rotacionado
            # Determina a magnitude da velocidade alvo com base no perfil
            current_profile_speed_magnitude = self._calculate_profile_speed(
                current_abs_angle_rotated, total_abs_angle_to_rotate,
                startspeed, maxspeed, endspeed,
                addspeed, 1 - brakeStart # 1 - brakeStart fornece a razão de desaceleração
            )
            # Aplica a direção geral do movimento (para frente/para trás)
            effective_speed = current_profile_speed_magnitude * movement_direction_sign
            # Calcula as potências reais dos motores e as limita entre -100 e 100
            left_power = max(min(int(effective_speed * left_speed_factor), 100), -100)
            right_power = max(min(int(effective_speed * right_speed_factor), 100), -100)
            # Aplica a potência aos motores
            left_motor.dc(left_power)
            right_motor.dc(right_power)
            # Atualiza a última leitura do giroscópio para a próxima iteração
            last_heading = current_heading
            # Verifica as condições de término
            if stopMethod is not None:
                if stopMethod.loop():
                    loop = False
                    break
            # Verifica se o ângulo alvo foi atingido
            if current_abs_angle_rotated >= total_abs_angle_to_rotate:
                loop = False
                break
            if abs(left_motor.angle() / 360) + abs(right_motor.angle() / 360) / 2 >= parametro:
                loop = False
                break
            wait(10) # Pequeno atraso para evitar loop apertado
        # Para os motores se solicitado
        if stop:
            left_motor.stop()
            right_motor.stop()
        return # Fim de arcRotation
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
    seguidor = LineFollower()
    seguidor.calculate_pid(start_speed=50, speed_oscilation=35)
if __name__ == '__main__':
    main()