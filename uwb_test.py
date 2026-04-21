import time
import math
import csv
# 假设你的文件叫 hardware_manager.py，请根据实际文件名修改
from sensors.uwb import DualUWBManager 

def run_uwb_accuracy_test(duration_sec=30, output_file="uwb_test_data.csv"):
    """
    专门为写 Report 准备的 UWB 精度数据采集脚本。
    运行指定时间，并把数据记录到 CSV 文件中。
    """
    # ==========================================
    # 1. 模拟初始化配置 (请根据你们的真实情况修改)
    # ==========================================
    config = {
        'hardware': {
            'uwb_port': '/dev/ttyUSB0',  # 树莓派上的串口，可能是 /dev/serial0 
            'uwb_baudrate': 115200
        }
    }
    L = 0.6  # [重要] UWB 左右天线的物理基线长度 (米)，务必填写真实值

    print("[*] 正在初始化 UWB 硬件...")
    uwb = DualUWBManager(config)
    
    # 给一点时间让串口建立连接，并让你的滑动平均滤波器充满初始数据
    print("[*] 等待滤波器稳定...")
    time.sleep(2.0) 

    print(f"[*] 开始采集数据，持续 {duration_sec} 秒...")
    print(f"[*] 数据将保存至: {output_file}")
    print("-" * 65)
    print(f"{'Time(s)':<10} | {'Left UWB(m)':<15} | {'Right UWB(m)':<15} | {'Target Angle(deg)':<15}")
    print("-" * 65)

    start_time = time.time()

    # 打开 CSV 文件准备写入
    with open(output_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        # 写入表头 (Report 所需的数据特征)
        writer.writerow(['Time_s', 'Left_Dist_m', 'Right_Dist_m', 'Target_Angle_deg'])

        # ==========================================
        # 2. 核心采样循环
        # ==========================================
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time > duration_sec:
                break
            
            # --- 数据 1: 单个 UWB 算出的距离 (已通过你的滤波器) ---
            dl, dr = uwb.get_distances()

            # --- 数据 2: 我们的算法算出的目标角度 ---
            # 复用你们极简 APF 里的三角定位几何逻辑
            target_y = (dr**2 - dl**2) / (2 * L)
            val_for_sqrt = dl**2 - (target_y - L/2)**2
            target_x = math.sqrt(max(val_for_sqrt, 0.001))
            
            # 计算角度 (弧度转角度，更直观方便写 Report)
            # 正前方是 0 度，左边是正角度，右边是负角度
            angle_rad = math.atan2(target_y, target_x)
            angle_deg = math.degrees(angle_rad)

            # 打印到终端实时观察
            print(f"{elapsed_time:<10.2f} | {dl:<15.4f} | {dr:<15.4f} | {angle_deg:<15.2f}")

            # 写入 CSV (保留 3 位小数即可)
            writer.writerow([round(elapsed_time, 3), round(dl, 3), round(dr, 3), round(angle_deg, 3)])

            # 采样频率控制 (0.1 秒 = 10Hz)
            time.sleep(0.1)

    print("-" * 65)
    print(f"[*] 测试完成！请拿着 {output_file} 去分析精度吧！")

if __name__ == "__main__":
    # 你可以修改测试时长，比如测 60 秒
    run_uwb_accuracy_test(duration_sec=30)