import math
import serial
import time

class MotorController:
    def __init__(self, config=None):
        """Initialize the motor controller by connecting to the Arduino via serial port"""
        print("[*] Connecting to motor controller...")

        # 1. Parameters for motion control
        self.wheel_base = 0.20  
        self.max_physical_speed = 1.0  
        
        # 2. Serial port configuration
        # Note: Arduino Uno plugged into Raspberry Pi is usually recognized as /dev/ttyACM0
        # If you are using Nano/Mega, it might be /dev/ttyUSB1
        self.port = '/dev/ttyACM0' 
        self.baudrate = 115200
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2) # Wait for Arduino to restart and be ready
            print(f"[*] Successfully connected to motor driver board ({self.port})")
        except Exception as e:
            print(f"[!] Warning: Unable to open motor serial port {self.port} - {e}")
            self.ser = None

    def send_cmd_vel(self, v, w):
        """Receive v and w, convert to PWM, and send to Arduino"""

        # Step 1: Inverse differential drive kinematics (calculate desired linear velocity)
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # Step 2: Map to Arduino PWM range of -255 to +255
        pwm_left = int((v_left / self.max_physical_speed) * 255)
        pwm_right = int((v_right / self.max_physical_speed) * 255)

        # Step 3: Limiting protection
        pwm_left = max(min(pwm_left, 255), -255)
        pwm_right = max(min(pwm_right, 255), -255)

        # Step 4: Concatenate string and send
        if self.ser:
            try:
                # Format: "150,-100\n"
                cmd_str = f"{pwm_left},{pwm_right}\n"
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                pass
                
        # Terminal monitoring print
        print(f"-> [Chassis Command] v:{v:.2f} w:{w:.2f} | Serial Send: L:{pwm_left} R:{pwm_right}")

    def stop_all(self):
        """Emergency stop"""
        if self.ser:
            self.ser.write("0,0\n".encode('utf-8'))