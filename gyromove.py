from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button
from pybricks.tools import wait, StopWatch
from configuracao import hub, left_motor, right_motor, wheel_diameter

class GyroMove:
    """Move o robô em linha reta por uma distância específica, mantendo heading zero com PID."""
    def __init__(self, kp=1.0, ki=0.0, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def move(self, distance_mm: float, speed: float, timeout=10000):
        """
        Move o robô uma distância (mm) à velocidade (positivo = frente).
        Usa o giroscópio para manter heading = 0 com PID.
        """
        # Reset sensores
        hub.imu.reset_heading(0)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        # PID estados
        integral = 0
        last_error = 0
        # Configurar tempo e direção
        timer = StopWatch()
        forward = 1 if distance_mm >= 0 else -1
        target = abs(distance_mm)

        while True:
            # Verifica timeout ou parada manual
            if timer.time() > timeout or Button.CENTER in hub.buttons.pressed():
                break

            # Calcula distância percorrida pelos motores (mm)
            avg_angle = (left_motor.angle() + right_motor.angle()) / 2
            dist = abs(avg_angle) / 360 * (3.14159 * wheel_diameter)
            if dist >= target:
                break

            # Erro de heading (queremos 0)
            current_heading = hub.imu.heading()
            error = current_heading  # alvo 0
            integral += error
            derivative = error - last_error

            # Saída PID
            correction = self.kp * error + self.ki * integral + self.kd * derivative
            last_error = error

            # Calcula potências dos motores
            power_left = speed * forward + correction
            power_right = speed * forward - correction
            # Limita para [-100, 100]
            power_left = max(-100, min(100, int(power_left)))
            power_right = max(-100, min(100, int(power_right)))

            # Aplica potência
            left_motor.dc(power_left)
            right_motor.dc(power_right)

            wait(10)

        # Finaliza movimento
        left_motor.brake()
        right_motor.brake()


gyro = GyroMove()
gyro.move(100, 100)