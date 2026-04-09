import math
from gpiozero import Motor

class MotorController:
    def __init__(self, config=None):
        """Initialize the motor controller"""
        print("[*] Initializing motor controller...")

        # --- 1. 机器人物理参数 (你可以后期把它们移到 config.yaml 里) ---
        # 左右轮子的中心间距 (单位：米)
        self.wheel_base = 0.20  
        # 小车在满功率 (100% PWM) 下的大致直线最大速度 (单位：m/s)
        # 这个值用于将算法速度映射为 PWM 占空比，需要实际下地测试微调
        self.max_physical_speed = 1.0  

        # --- 2. 初始化 L298N 引脚 (使用 BCM 编码，需根据你的实际接线修改) ---
        # 左侧电机: IN1=5, IN2=6, ENA=12
        self.motor_left = Motor(forward=5, backward=6, enable=12)
        
        # 右侧电机: IN3=19, IN4=26, ENB=13
        self.motor_right = Motor(forward=19, backward=26, enable=13)

    def send_cmd_vel(self, v, w):
        """
        Receive the velocity commands from the algorithm and convert them to PWM signals for the L298N
        :param v: Linear velocity (m/s), positive forward
        :param w: Angular velocity (rad/s), positive counterclockwise (left turn)
        """
        # --- Step 1: Inverse Differential Kinematics ---
        # Calculate the expected linear velocities of the left and right wheels
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # --- Step 2: Speed -> PWM Duty Cycle Mapping ---
        # Convert m/s to PWM duty cycle ratio between -1.0 and +1.0
        pwm_left = v_left / self.max_physical_speed
        pwm_right = v_right / self.max_physical_speed

        # --- Step 3: Clamping to Prevent Errors ---
        # gpiozero requires duty cycle to be between -1.0 and 1.0, otherwise it will crash
        pwm_left = max(min(pwm_left, 1.0), -1.0)
        pwm_right = max(min(pwm_right, 1.0), -1.0)

        # --- Step 4: Drive L298N ---
        # Control left motor
        if pwm_left > 0:
            self.motor_left.forward(pwm_left)    # Forward
        elif pwm_left < 0:
            self.motor_left.backward(abs(pwm_left)) # Backward
        else:
            self.motor_left.stop()               # Stop

        # Control right motor
        if pwm_right > 0:
            self.motor_right.forward(pwm_right)
        elif pwm_right < 0:
            self.motor_right.backward(abs(pwm_right))
        else:
            self.motor_right.stop()

        # Terminal monitoring print (can be commented out when running on the robot to avoid screen flickering)
        print(f"-> [Chassis] v:{v:.2f} w:{w:.2f} | PWM_L: {pwm_left:.2f} PWM_R: {pwm_right:.2f}")

    def stop_all(self):
        """Emergency stop"""
        self.motor_left.stop()
        self.motor_right.stop()