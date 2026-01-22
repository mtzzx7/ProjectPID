from config import *
 # Importado para potenciais usos futuros, embora não estritamente necessário para esta implementação


def normaliza(reflection, preto, branco):
    """Normaliza leitura de refletância para 0..100 usando valores de calibração."""
    if branco == preto:
        return 0
    val = (reflection - preto) / (branco - preto) * 100
    if val < 0:
        return 0
    if val > 100:
        return 100
    return val


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
        self.kp = 1.123 # proporcional
        self.ki = 0.001 # integral
        self.kd = 0.5 # derivativo
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
        # Parâmetros para detecção de oscilação
        self.osc_noise_threshold = 2.0
        self.osc_fraction_threshold = 0.4
        self.osc_amplitude_threshold = 5.0
        self.osc_derivative_threshold = 8.0
        self.timer = StopWatch()
        self.dt = 0
            # Controle de suavização e proteção contra respostas agressivas
        self.deriv_limit = 50.0
        self.correction_alpha = 0.45  # suavização low-pass para a correção (0..1)
            # Parâmetros de recuperação em gap (ambos sensores em preto)
        self.gap_recovery_speed_factor = 0.45  # reduz velocidade quando encontra gap
        self.gap_dampen_factor = 0.35  # reduz correção instantânea em gap
        self.gap_hold_ms = 80  # tempo pequeno para estabilizar ao detectar gap
        self.last_correction = 0.0

    def detect_oscillation(self):
        """Detecta oscilação de forma robusta.

        Estratégia:
        - Ignora pequenas variações abaixo de `osc_noise_threshold`.
        - Conta cruzamentos de sinal apenas entre pontos com magnitude relevante.
        - Calcula amplitude (peak-to-peak) e média absoluta da derivada.
        - Declara oscilação quando há muitos cruzamentos (fração >= osc_fraction_threshold)
          e (amplitude >= osc_amplitude_threshold ou derivada média >= osc_derivative_threshold).
        """
        if len(self.error_history) < 2:
            return False

        # parâmetros locais (podem ser ajustados via atributos da instância)
        noise_th = getattr(self, 'osc_noise_threshold', 2.0)
        frac_th = getattr(self, 'osc_fraction_threshold', 0.4)
        amp_th = getattr(self, 'osc_amplitude_threshold', 5.0)
        deriv_th = getattr(self, 'osc_derivative_threshold', 8.0)

        eh = list(self.error_history)

        # Conta cruzamentos relevantes e calcula derivadas
        sign_changes = 0
        total_pairs = 0
        abs_derivs = []
        for i in range(1, len(eh)):
            a = eh[i-1]
            b = eh[i]
            total_pairs += 1
            # só conta cruzamento se ambos excedem o ruído
            if abs(a) > noise_th and abs(b) > noise_th:
                if a * b < 0:
                    sign_changes += 1
            abs_derivs.append(abs(b - a))

        zero_cross_frac = sign_changes / max(1, total_pairs)
        peak_to_peak = max(eh) - min(eh)
        avg_abs_deriv = sum(abs_derivs) / len(abs_derivs) if abs_derivs else 0

        if zero_cross_frac >= frac_th and (peak_to_peak >= amp_th or avg_abs_deriv >= deriv_th):
            return True
        return False
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
        
    def scan(self, power=30, sample_ms=50, direction='left', debug=False):
        """
        Gira ~360 graus no lugar e coleta leituras de refletância dos sensores de cor.

        Args:
            power (int): potência de giro (0-100). Valor positivo usa `direction` para escolher sentido.
            sample_ms (int): intervalo entre amostras em ms.
            direction (str): 'left' ou 'right' — sentido da rotação.
            debug (bool): se True, imprime estatísticas parciais durante a varredura.

        Returns:
            dict: estatísticas das leituras {
                'left': {'count', 'min', 'max', 'mean', 'std'},
                'right': {...},
                'samples': total amostras,
            }
        """
        # Escolhe sinais de potência para girar no lugar
        sign = 1 if direction == 'left' else -1
        left_power = -sign * abs(power)
        right_power = sign * abs(power)

        left_vals = []
        right_vals = []

        # Zera heading de referência e começa a girar
        try:
            last_h = hub.imu.heading()
        except Exception:
            last_h = 0

        accumulated = 0.0
        # Inicia giro
        left_motor.dc(left_power)
        right_motor.dc(right_power)

        try:
            while accumulated < 360.0:
                # Amostra reflexões
                try:
                    lv = color_sensor_esquerdo.reflection()
                    rv = color_sensor_direito.reflection()
                except Exception:
                    # Fallback: zeros se sensor não disponível
                    lv, rv = 0, 0
                left_vals.append(lv)
                right_vals.append(rv)

                # Atualiza heading e acumula delta absoluto
                try:
                    h = hub.imu.heading()
                    delta = ((h - last_h + 540) % 360) - 180
                    accumulated += abs(delta)
                    last_h = h
                except Exception:
                    # Se não houver IMU, rota por tempo aproximado: usa sample_ms
                    accumulated += (abs(power) / 100.0) * (sample_ms / 1000.0) * 360.0 * 0.01

                if debug and (len(left_vals) % max(1, int(500 / sample_ms)) == 0):
                    # imprime a cada ~500ms
                    try:
                        print(f"scan: acc={accumulated:.1f}°, samples={len(left_vals)}")
                    except Exception:
                        pass

                wait(sample_ms)

        finally:
            # Garante parada dos motores
            left_motor.brake()
            right_motor.brake()

        # Estatísticas
        def stats(arr):
            n = len(arr)
            if n == 0:
                return {'count': 0, 'min': 0, 'max': 0, 'mean': 0, 'std': 0}
            smin = min(arr)
            smax = max(arr)
            mean = sum(arr) / n
            var = sum((x - mean) ** 2 for x in arr) / n
            std = var ** 0.5
            return {'count': n, 'min': smin, 'max': smax, 'mean': mean, 'std': std}

        result = {
            'left': stats(left_vals),
            'right': stats(right_vals),
            'samples': len(left_vals)
        }

        # Aplica calibração direta na instância para uso em calculate_pid
        # calcula min/max coletados e expande caso a faixa seja muito pequena
        left_stats = result['left']
        right_stats = result['right']

        lmin = left_stats.get('min', 0)
        lmax = left_stats.get('max', 100)
        rmin = right_stats.get('min', 0)
        rmax = right_stats.get('max', 100)

        # Se faixa muito pequena (sensor não detectou contraste), adiciona margem
        if (lmax - lmin) < 5:
            mid = (lmax + lmin) / 2
            lmin = max(0, mid - 10)
            lmax = min(100, mid + 10)
        if (rmax - rmin) < 5:
            mid = (rmax + rmin) / 2
            rmin = max(0, mid - 10)
            rmax = min(100, mid + 10)

        # assegura atributos mesmo que arrays vazios
        self.reflexo_preto_esq = lmin
        self.reflexo_branco_esq = lmax
        self.reflexo_preto_dir = rmin
        self.reflexo_branco_dir = rmax
        self.is_calibrated = True
        if debug:
            try:
                print('Calibração aplicada (instância):')
                print(f" ESQ min/max = {self.reflexo_preto_esq}/{self.reflexo_branco_esq}")
                print(f" DIR min/max = {self.reflexo_preto_dir}/{self.reflexo_branco_dir}")
            except Exception:
                pass

        if debug:
            try:
                print('Scan 360 done:', result)
            except Exception:
                pass

        return result

    def pid(self, start_speed, speed_oscilation):
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
            # 1. Normalização dos sensores (MOVIDO PARA CIMA)
            try:
                # Normaliza entre 0 e 100 baseado nas leituras calibradas
                self.norm_dir = normaliza(color_sensor_direito.reflection(), self.reflexo_preto_dir, self.reflexo_branco_dir)
                self.norm_esq = normaliza(color_sensor_esquerdo.reflection(), self.reflexo_preto_esq, self.reflexo_branco_esq)
            except Exception:
                # Se não calibrado ou erro, usa leitura direta (fallback)
                self.norm_esq = color_sensor_esquerdo.reflection()
                self.norm_dir = color_sensor_direito.reflection()

            # 2. Detecção de eventos (agora com os valores normalizados disponíveis)
            if self.detect_oscillation():
                print("Oscilação detectada!")

            curve = self.detect_sharp_curve()

            # 3. Cálculo do PID
            self.dt = self.timer.time() / 1000 
            self.timer.reset()

            if self.dt <= 0:
                self.dt = 0.01  # proteção
            
            self.error = self.norm_esq - self.norm_dir
            self.proporcional = self.kp * self.error
            self.integral += self.error * self.dt
            # derivada protegida contra picos
            raw_deriv = (self.error - self.last_error) / self.dt
            # limita derivada
            if raw_deriv > self.deriv_limit:
                raw_deriv = self.deriv_limit
            elif raw_deriv < -self.deriv_limit:
                raw_deriv = -self.deriv_limit
            self.derivative = raw_deriv
            self.last_error = self.error
            self.error_history.append(self.error)
            if len(self.error_history) > self.max_history_length:
                self.error_history.pop(0)

            # Anti-windup para integral
            integral_limit = 500
            if self.integral > integral_limit:
                self.integral = integral_limit
            elif self.integral < -integral_limit:
                self.integral = -integral_limit

            # 4. Lógica de Ação baseada em eventos
            if curve == "sharp_left":
                print("Curva acentuada para a esquerda detectada!")
                self.integral *= 0.5 # Reduz o integral para não exagerar na saída da curva
                self.speed = speed_oscilation
            elif curve == "sharp_right":
                print("Curva acentuada para a direita detectada!")
                self.integral *= 0.5
                self.speed = speed_oscilation
            else:
                self.speed = start_speed 
            
            # 5. Cálculo final da correção e aplicação aos motores
            # cálculo original do PID
            correction = self.proporcional + (self.ki * self.integral) + (self.kd * self.derivative)

            # Detecta gap: ambos sensores em preto -> comportamento mais suave
            gap = (self.norm_esq < self.black_threshold) and (self.norm_dir < self.black_threshold)
            if gap:
                self.qnt_pretos += 1
                # reduz velocidade para recuperar
                self.speed = min(self.speed, start_speed * self.gap_recovery_speed_factor)
                # amortecer correção para evitar saída agressiva
                correction = self.last_correction * (1.0 - self.gap_dampen_factor)
                # pequena pausa para estabilizar
                try:
                    wait(self.gap_hold_ms)
                except Exception:
                    pass
            else:
                # reseta contador quando volta a ver linha
                self.qnt_pretos = 0

            # Suaviza a correção (low-pass entre iterações)
            correction = (self.last_correction * (1.0 - self.correction_alpha)) + (correction * self.correction_alpha)
            self.last_correction = correction

            left_power = self.speed - correction
            right_power = self.speed + correction
            left_motor.dc(left_power)
            right_motor.dc(right_power)            
            """
            if ultra.distance() <= 50:
                gyro_move_universal("angulo", -35, 50)
                gyrouniversal(90)
                seguidor = LineFollower()
                seguidor.arcRotation(100, -100, 35, 60, 35, addspeed=0.3, brakeStart=0.7, stopMethod=None, generator=None, stop=True, parametro=7)
                gyrouniversal(-90)
                wait(1000)
                gyro_move_universal("dois_pretos", 45)
                gyro_move_universal("angulo", 35, 10)
                gyrouniversal(90)
            """
            wait(10)
        
        left_motor.stop()
        right_motor.stop()



