import serial
import threading
import time
from collections import deque
import statistics

class MedianFilter:
    """[修改] 中值滤波器，专门用于剔除 UWB 偶尔跳出的极大/极小噪点"""
    def __init__(self, window_size=5):
        self._buf = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        self._buf.append(value)
        return statistics.median(self._buf)

class DualUWBManager:
    def __init__(self, config):
        print("[*] Initializing UWB ports...")
        self.dl = 2.0 
        self.dr = 2.0 
        
        # [新增] 实例化两个中值滤波器
        self.filter_l = MedianFilter(window_size=5)
        self.filter_r = MedianFilter(window_size=5)
        
        self.port = config['hardware']['uwb_port']
        # ... 后续串口初始化代码保持不变 ...

    def _read_loop(self):
        """Background loop to parse L and R data..."""
        while True:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if "+ANCHOR_RCV=" in line:
                        parts = line.split(',')
                        if len(parts) >= 4:
                            dist_str = parts[3]
                            dist_cm_clean = dist_str.replace('cm', '').strip()
                            
                            # [修改] 使用 try-except 替代 isdigit，增强健壮性
                            try:
                                raw_dist_m = float(dist_cm_clean) / 100.0
                                # [新增] 将原始数据放入滤波器，获取平滑后的数据
                                if "Anchor1" in line:
                                    self.dl = self.filter_l.update(raw_dist_m)
                                elif "Anchor2" in line:
                                    self.dr = self.filter_r.update(raw_dist_m)
                            except ValueError:
                                pass # 丢弃无法转换的乱码
                except Exception as e:
                    pass
            time.sleep(0.01)

    def get_distances(self):
        return self.dl, self.dr