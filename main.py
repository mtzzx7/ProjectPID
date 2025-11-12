from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button
from pybricks.tools import wait, StopWatch
from configuracao import hub, left_motor, right_motor

class GyroTurn:
    """Gira o robô até um ângulo alvo usando o giroscópio com controle PID."""
    def __init__(self, kp=2.0, ki=0.001, kd=0.3, tolerance=2.0, max_power=60):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tolerance = tolerance  # tolerância de ângulo (graus)
        self.max_power = max_power  # potência máxima permitida
        self.reset_pid()

    def reset_pid(self):
        """Reseta estados internos do PID."""
        self.integral = 0
        self.last_error = 0

    def normalize_angle(self, target, current):
        """Calcula o menor ângulo de target- current em ±180°:contentReference[oaicite:6]{index=6}."""
        error = ((target - current + 540) % 360) - 180
        return error

    def turn_to(self, angle: float, timeout=5000):
        """
        Gira o robô para o ângulo (graus) relativo ao heading inicial.
        Usa PID para controle suave e corrige overshoot ao final.
        """
        # Reset giroscópio e PID
        hub.imu.reset_heading(0)
        self.reset_pid()
        target = angle
        timer = StopWatch()

        while True:
            # Checa timeout ou botão de emergência
            if timer.time() > timeout or Button.CENTER in hub.buttons.pressed():
                break

            # Calcula erro angular atual
            current = hub.imu.heading()
            error = self.normalize_angle(target, current)

            # Sai do loop se estiver dentro da tolerância
            if abs(error) <= self.tolerance:
                break

            # Atualiza integral e derivativo
            self.integral += error
            derivative = error - self.last_error

            # PID
            power = self.kp * error + self.ki * self.integral + self.kd * derivative
            # Limita potência
            power = max(-self.max_power, min(self.max_power, power))

            # Aplica potência invertida nos motores para girar
            if power > 0:
                left_motor.dc(-abs(power))
                right_motor.dc(abs(power))
            else:
                left_motor.dc(abs(power))
                right_motor.dc(-abs(power))

            self.last_error = error
            wait(10)

        # Parada suave e pulse corretivo se necessário:contentReference[oaicite:7]{index=7}
        left_motor.hold()
        right_motor.hold()
        # Pequeno ajuste final se ainda houver erro residual
        residual = self.normalize_angle(target, hub.imu.heading())
        if abs(residual) > self.tolerance:
            pulse = 40
            if residual > 0:
                left_motor.dc(-pulse)
                right_motor.dc(pulse)
            else:
                left_motor.dc(pulse)
                right_motor.dc(-pulse)
            wait(100)
            left_motor.hold()
            right_motor.hold()


gyro = GyroTurn()
gyro.turn_to(90)