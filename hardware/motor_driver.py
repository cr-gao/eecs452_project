import math
from gpiozero import Motor

class MotorController:
    def __init__(self, config=None):
        """Init L298N motor driver with given configuration"""
        print("[*] Initializing L298N motor driver...")

        # --- 1. Configuration ---
        # The wheel_base is the distance between the left and right wheels (in meters)
        self.wheel_base = 0.20  
        # The maximum physical speed of the robot (in m/s) at full PWM (100%)
        # This value is used to map algorithmic speeds to PWM duty cycles, needs to be tuned through real-world testing
        self.max_physical_speed = 1.0

        # --- 2. Initialize L298N pins (using BCM numbering, modify according to your actual wiring) ---
        # Left motor: IN1=5, IN2=6, ENA=12
        self.motor_left = Motor(forward=5, backward=6, enable=12)
        
        # Right motor: IN3=19, IN4=26, ENB=13
        self.motor_right = Motor(forward=19, backward=26, enable=13)

    def send_cmd_vel(self, v, w):
        """
        Receive algorithmic velocity commands and convert them to PWM signals for L298N
        :param v: Linear velocity (m/s), positive forward
        :param w: Angular velocity (rad/s), positive counter-clockwise (left turn)
        """
        # --- Step 1: Differential Drive Kinematics Inverse ---
        # Calculate the desired linear velocities for each wheel
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # --- Step 2: Speed -> PWM Duty Cycle Mapping ---
        # Convert m/s to a PWM ratio between -1.0 and +1.0
        pwm_left = v_left / self.max_physical_speed
        pwm_right = v_right / self.max_physical_speed

        # --- Step 3: Clamping to Prevent Errors ---
        # gpiozero requires the duty cycle to be between -1.0 and +1.0, otherwise it will crash
        pwm_left = max(min(pwm_left, 1.0), -1.0)
        pwm_right = 0.85 * max(min(pwm_right, 1.0), -1.0)

        # --- Step 4: Drive L298N ---
        # Control the left motor
        if pwm_left > 0:
            self.motor_left.forward(pwm_left)    # Forward
        elif pwm_left < 0:
            self.motor_left.backward(abs(pwm_left)) # Backward
        else:
            self.motor_left.stop()               # Stop

        # Control the right motor
        if pwm_right > 0:
            self.motor_right.forward(pwm_right)
        elif pwm_right < 0:
            self.motor_right.backward(abs(pwm_right))
        else:
            self.motor_right.stop()

        # Terminal monitoring print (can be commented out during actual operation to avoid screen flickering)
        print(f"-> [Chassis] v:{v:.2f} w:{w:.2f} | PWM_L: {pwm_left:.2f} PWM_R: {pwm_right:.2f}")

    def stop_all(self):
        """Emergency stop"""
        self.motor_left.stop()
        self.motor_right.stop()
