import serial
import threading
import time

class DualUWBManager:
    def __init__(self, config):
        """Initialize the dual UWB serial connections and start background threads for reading data"""
        print("[*] Initializing dual UWB serial connections...")
        self.dl = 2.0 # Default left distance
        self.dr = 2.0 # Default right distance
        
        self.baudrate = config['hardware']['uwb_baudrate']
        self.port_left = config['hardware']['uwb_left_port']
        self.port_right = config['hardware']['uwb_right_port']
        
        self.ser_left = self._open_serial(self.port_left)
        self.ser_right = self._open_serial(self.port_right)
        
        # Launch background threads to continuously read from the UWB serial ports
        threading.Thread(target=self._read_loop, args=(self.ser_left, True), daemon=True).start()
        threading.Thread(target=self._read_loop, args=(self.ser_right, False), daemon=True).start()

    def _open_serial(self, port):
        try:
            return serial.Serial(port, self.baudrate, timeout=0.1)
        except Exception as e:
            print(f"[!] Warning: Unable to open UWB serial port {port} - {e}")
            return None

    def _read_loop(self, ser, is_left):
        """Background thread function for polling serial data"""
        while True:
            if ser and ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line.startswith("UWB:"):
                        dist_m = float(line.split(":")[1]) / 100.0
                        if is_left:
                            self.dl = dist_m
                        else:
                            self.dr = dist_m
                except Exception:
                    pass
            time.sleep(0.01)

    def get_distances(self):
        """Return the latest parsed left and right UWB distances (unit: meters)"""
        return self.dl, self.dr