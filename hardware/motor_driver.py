class MotorController:
    def __init__(self):
        """Initialize the motor controller, set up PWM or serial communication as needed"""
        print("[*] Initializing motor controller...")
        # Config with concrete motor driver initialization code here

    def send_cmd_vel(self, v, w):
        """
        Send velocity commands to the motor driver
        :param v: Linear velocity (m/s)
        :param w: Angular velocity (rad/s)
        """
        # --- [TODO] Add motor driver control code here ---
        
        # For demonstration, we just print the commands. Replace this with actual motor control code.
        print(f"-> [底盘指令] 线速度 v: {v:.2f} m/s | 角速度 w: {w:.2f} rad/s")