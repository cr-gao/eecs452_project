import serial
import threading
import time

class DualUWBManager:
    def __init__(self, config):
        """Initialize double uwb with a single esp32"""
        print("[*] Initializing UWB ports...")
        self.dl = 2.0 
        self.dr = 2.0 
        
        self.port = config['hardware']['uwb_port']
        self.baudrate = config['hardware']['uwb_baudrate']
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            # Single-thread reading
            threading.Thread(target=self._read_loop, daemon=True).start()
        except Exception as e:
            print(f"[!] Failed to open port {self.port} - {e}")
            self.ser = None

    def _read_loop(self):
        """Process L/R data in turn"""
        while True:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    # 分流判断
                    if line.startswith("UWB_L:"):
                        dist_m = float(line.split(":")[1]) / 100.0
                        self.dl = dist_m
                    elif line.startswith("UWB_R:"):
                        dist_m = float(line.split(":")[1]) / 100.0
                        self.dr = dist_m
                except Exception:
                    pass
            time.sleep(0.01)

    def get_distances(self):
        return self.dl, self.dr