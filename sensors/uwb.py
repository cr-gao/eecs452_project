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
        """Background loop to parse L and R data from a single serial port"""
        while True:
            if self.ser and self.ser.in_waiting:
                try:
                    # Read a line of data, such as "Anchor2 received: +ANCHOR_RCV=TAG, 0,, 16cm, -52"
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    # Make sure the line contains the expected pattern before parsing
                    if "+ANCHOR_RCV=" in line:
                        # Split the line by commas, which should give us parts like ['Anchor2 received: +ANCHOR_RCV=TAG', ' 0', '', ' 16cm', ' -52']
                        parts = line.split(',')
                        
                        if len(parts) >= 4:
                            # Extract the part containing 'cm', e.g., ' 16cm'
                            dist_str = parts[3]
                            # Remove 'cm' and whitespace, keeping only the numeric value '16'
                            dist_cm_clean = dist_str.replace('cm', '').strip()
                            
                            if dist_cm_clean.isdigit(): # Make sure it's a valid number before converting
                                dist_m = float(dist_cm_clean) / 100.0 # Convert to meters
                                
                                # Determine if it's Anchor1 (left) or Anchor2 (right)
                                if "Anchor1" in line:
                                    self.dl = dist_m
                                elif "Anchor2" in line:
                                    self.dr = dist_m
                except Exception as e:
                    # Ignore parsing errors (e.g., serial port noise)
                    pass
            time.sleep(0.01)

    def get_distances(self):
        return self.dl, self.dr